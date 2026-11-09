#!/usr/bin/env python3
"""
Clasificador de botellas por peso (HX711 + servo SG90).

  - Peso entre 10 y 50 g -> ACEPTADA (reciclaje)  -> servo.min()
  - Otro peso            -> RECHAZADA (basura)    -> servo.max()

Uso:
  python3 codigo_pi3.py --calibrar   (solo la primera vez)
  python3 codigo_pi3.py --distancia  (probar el HC-SR04)
  python3 codigo_pi3.py              (funcionamiento normal)
"""

import argparse
import json
import statistics
import time
from pathlib import Path

import RPi.GPIO as GPIO
from gpiozero import Servo

# ===================== CONFIGURACION =====================
PIN_SERVO = 18
PIN_HX711_DT = 5      # BCM (pin fisico 29)
PIN_HX711_SCK = 6     # BCM (pin fisico 31)
PIN_TRIG = 23         # BCM (pin fisico 16)  HC-SR04 TRIG
PIN_ECHO = 24         # BCM (pin fisico 18)  HC-SR04 ECHO (con divisor de voltaje!)

PESO_MIN_RECICLAJE = 10.0   # g
PESO_MAX_RECICLAJE = 50.0   # g
PESO_MIN_PRESENCIA = 5.0    # bajo este peso = plataforma vacia

TIEMPO_ESTABLE = 1.0        # s con peso estable antes de decidir
TOLERANCIA_ESTABLE = 3.0    # g de variacion maxima permitida
LECTURAS_ESTABLES = 4       # lecturas seguidas parecidas para decidir
TIEMPO_DESCARGA = 1.5       # s que el servo sostiene la posicion

# Recorrido del servo. El SG90 admite ~0.5 a 2.4 ms (casi 180 grados).
# Si choca o se pasa del limite, acerca estos valores (ej. 0.6 y 2.3).
PULSO_MIN_MS = 0.5
PULSO_MAX_MS = 2.4
# Cuanto gira hacia cada lado desde el centro: de 0.0 (nada) a 1.0 (maximo).
VALOR_RECICLAJE = -1.0   # negativo = un lado
VALOR_BASURA = 1.0       # positivo = el otro lado

# Sensor ultrasonico: verifica que el objeto ya no este en la plataforma.
MARGEN_DETECCION_CM = 3.0   # si lee mas de 3 cm MENOS que la plataforma vacia = hay objeto
TIEMPO_CAIDA = 1.0          # s de espera tras mover el servo antes de verificar
REANUDAR_AL_RETIRAR = True  # True: sigue solo al retirar el objeto; False: el programa termina

ARCHIVO_CALIBRACION = Path(__file__).with_name("calibracion.json")
# =========================================================


# ---------- Servo (igual que tu codigo original) ----------
def crear_servo():
    # Sin pigpio: usa el metodo por defecto de gpiozero (funciona en Pi 3)
    return Servo(PIN_SERVO, min_pulse_width=PULSO_MIN_MS/1000,
                 max_pulse_width=PULSO_MAX_MS/1000)


servo = crear_servo()


def posicion_inicial():
    print("Servo en espera...")
    servo.mid()
    time.sleep(0.5)
    servo.value = None  # evita vibracion y consumo mientras espera


def aceptar_botella():
    print("--> Botella ACEPTADA (reciclaje)")
    servo.value = VALOR_RECICLAJE
    time.sleep(TIEMPO_DESCARGA)
    posicion_inicial()


def rechazar_botella():
    print("--> Botella RECHAZADA (basura)")
    servo.value = VALOR_BASURA
    time.sleep(TIEMPO_DESCARGA)
    posicion_inicial()


# ---------- HX711 ----------
class HX711:
    def __init__(self, dt, sck):
        self.dt, self.sck = dt, sck
        self.offset = 0.0
        self.factor = 1.0  # cuentas por gramo
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.sck, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dt, GPIO.IN)

    def _leer_crudo(self, timeout=1.0):
        inicio = time.time()
        while GPIO.input(self.dt) == 1:
            if time.time() - inicio > timeout:
                raise TimeoutError("HX711 no responde (revisa el cableado)")
            time.sleep(0.001)
        valor = 0
        for _ in range(24):
            GPIO.output(self.sck, True)
            GPIO.output(self.sck, False)
            valor = (valor << 1) | GPIO.input(self.dt)
        GPIO.output(self.sck, True)   # pulso 25: canal A, ganancia 128
        GPIO.output(self.sck, False)
        if valor & 0x800000:
            valor -= 1 << 24
        return valor

    def leer_cuentas(self, muestras=3):
        return statistics.median(self._leer_crudo() for _ in range(muestras))

    def tara(self):
        self.offset = self.leer_cuentas(15)

    def leer_gramos(self):
        return (self.leer_cuentas() - self.offset) / self.factor


# ---------- HC-SR04 ----------
class HCSR04:
    """Sensor ultrasonico. Mide distancia en cm."""

    def __init__(self, trig, echo):
        self.trig, self.echo = trig, echo
        self.base = None  # distancia con la plataforma vacia
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.echo, GPIO.IN)
        time.sleep(0.1)

    def _medir_una(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)               # pulso de 10 us
        GPIO.output(self.trig, False)

        limite = time.perf_counter() + 0.03
        while GPIO.input(self.echo) == 0:
            if time.perf_counter() > limite:
                return None
        inicio = time.perf_counter()
        while GPIO.input(self.echo) == 1:
            if time.perf_counter() > limite:
                return None
        fin = time.perf_counter()
        return (fin - inicio) * 34300 / 2  # velocidad del sonido, ida y vuelta

    def distancia(self, muestras=5):
        """Mediana de varias lecturas. None si el sensor no responde."""
        datos = []
        for _ in range(muestras):
            d = self._medir_una()
            if d is not None and 2 <= d <= 400:
                datos.append(d)
            time.sleep(0.06)
        if len(datos) < 3:
            return None
        return statistics.median(datos)

    def medir_base(self):
        """Guarda la distancia con la plataforma vacia."""
        self.base = self.distancia(9)
        return self.base

    def hay_objeto(self):
        """True = hay objeto, False = libre, None = el sensor no responde."""
        d = self.distancia()
        if d is None:
            return None
        return d < self.base - MARGEN_DETECCION_CM


def calibrar(hx):
    print("== CALIBRACION ==")
    input("1) Deja la plataforma VACIA y presiona Enter... ")
    hx.tara()
    peso = float(input("2) Coloca un objeto de peso conocido e ingresa su peso en gramos: "))
    input("   Presiona Enter cuando este colocado y quieto... ")
    hx.factor = (hx.leer_cuentas(25) - hx.offset) / peso
    ARCHIVO_CALIBRACION.write_text(json.dumps({"factor": hx.factor}))
    print(f"Calibracion guardada (factor = {hx.factor:.2f})")


# ---------- Logica de decision ----------
def esperar_peso_estable(hx):
    """Espera un objeto con peso estable (N lecturas seguidas parecidas)."""
    ultimas = []
    while True:
        peso = hx.leer_gramos()
        if peso < PESO_MIN_PRESENCIA:
            ultimas.clear()
        else:
            ultimas.append(peso)
            ultimas = ultimas[-LECTURAS_ESTABLES:]
            if (len(ultimas) == LECTURAS_ESTABLES
                    and max(ultimas) - min(ultimas) <= TOLERANCIA_ESTABLE):
                return statistics.mean(ultimas)


def esperar_plataforma_vacia(hx):
    while hx.leer_gramos() >= PESO_MIN_PRESENCIA:
        time.sleep(0.2)
    time.sleep(0.5)


def verificar_retiro(sensor):
    """Comprueba con el HC-SR04 que el objeto ya no esta en la plataforma.
    Si sigue ahi, detiene todo y pide retirarlo manualmente.
    Devuelve True si se puede continuar, False si hay que terminar."""
    time.sleep(TIEMPO_CAIDA)
    while True:
        estado = sensor.hay_objeto()
        if estado is False:
            return True
        if estado is None:
            print("ERROR: el sensor HC-SR04 no responde. Revisa el cableado.")
        else:
            print("ERROR: el objeto SIGUE en la plataforma. "
                  "Sistema detenido: RETIRALO MANUALMENTE.")
        if not REANUDAR_AL_RETIRAR:
            return False
        time.sleep(2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrar", action="store_true")
    parser.add_argument("--medir", action="store_true",
                        help="muestra el peso en vivo (para diagnostico)")
    parser.add_argument("--distancia", action="store_true",
                        help="muestra la distancia del HC-SR04 en vivo")
    args = parser.parse_args()

    hx = HX711(PIN_HX711_DT, PIN_HX711_SCK)
    sensor = HCSR04(PIN_TRIG, PIN_ECHO)
    try:
        if args.calibrar:
            calibrar(hx)
            return

        if args.distancia:
            print("Distancia (Ctrl+C para salir):")
            while True:
                d = sensor.distancia()
                print("sin respuesta" if d is None else f"{d:6.1f} cm")
                time.sleep(0.3)

        if args.medir:
            if not ARCHIVO_CALIBRACION.exists():
                print("Falta calibrar primero.")
                return
            hx.factor = json.loads(ARCHIVO_CALIBRACION.read_text())["factor"]
            print("Haciendo tara con plataforma vacia...")
            time.sleep(1)
            hx.tara()
            print("Midiendo (Ctrl+C para salir):")
            while True:
                print(f"{hx.leer_gramos():8.1f} g")

        if not ARCHIVO_CALIBRACION.exists():
            print("Falta calibrar. Ejecuta: python3 codigo_pi3.py --calibrar")
            return
        hx.factor = json.loads(ARCHIVO_CALIBRACION.read_text())["factor"]

        posicion_inicial()
        print("Retira todo de la plataforma, haciendo tara...")
        time.sleep(1)
        hx.tara()
        if sensor.medir_base() is None:
            print("ERROR: el HC-SR04 no responde. Revisa el cableado "
                  "(prueba con --distancia).")
            return
        print(f"Plataforma vacia a {sensor.base:.1f} cm del sensor.")
        print("Listo. Coloca una botella (Ctrl+C para salir).")

        while True:
            peso = esperar_peso_estable(hx)
            print(f"Peso detectado: {peso:.1f} g")

            if PESO_MIN_RECICLAJE <= peso <= PESO_MAX_RECICLAJE:
                aceptar_botella()
            else:
                rechazar_botella()

            if not verificar_retiro(sensor):
                return
            esperar_plataforma_vacia(hx)
            print("Listo para la siguiente botella.")

    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        servo.value = None  # desactiva la senal del servo
        GPIO.cleanup()


if __name__ == "__main__":
    main()
