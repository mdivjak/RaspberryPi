import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

reader = SimpleMFRC522()

try:
    # text = "marko"
    # print("Place tag")
    # reader.write(text)
    # print("Written")
    id, text = reader.read()
    print(id)
    print(text)
finally:
    GPIO.cleanup()