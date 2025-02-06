# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 19:16:44 2025

@author: Jan
"""

import serial
import csv

# Ustawienia portu szeregowego
port = "COM5"  # Ustaw odpowiedni port COM dla Arduino
baudrate = 9600
output_file = "odczyt_pid_grzalka_wiatrak.csv"

# Otwórz połączenie z Arduino
with serial.Serial(port, baudrate, timeout=1) as ser, open(output_file, "w", newline="") as csvfile:
    csvwriter = csv.writer(csvfile)
    
    # Odczyt nagłówków z Arduino
    headers = ser.readline().decode("utf-8").strip().split(",")
    csvwriter.writerow(headers)  # Zapis nagłówków do pliku CSV

    print("Zapis danych do pliku rozpoczęty...")
    while True:
        try:
            # Odczyt danych z Arduino
            line = ser.readline().decode("utf-8").strip()
            if line:
                print(line)  # Wyświetl dane w konsoli
                data = line.split(",")
                csvwriter.writerow(data)  # Zapis danych do pliku CSV
        except KeyboardInterrupt:
            print("Zatrzymano zapis danych.")
            break
