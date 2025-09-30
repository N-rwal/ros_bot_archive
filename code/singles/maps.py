import serial
import time
import pygame
import threading
import sys

# === CONFIGURATION ===
SERIAL_PORT = "/dev/ttyUSB3"  # Change si nécessaire
BAUDRATE = 115200
REFRESH_INTERVAL = 5  # secondes entre lectures

def nmea_to_decimal(coord, direction):
    if not coord or coord == "":
        return None
    deg_len = 2 if direction in ['N', 'S'] else 3
    deg = int(coord[:deg_len])
    minutes = float(coord[deg_len:])
    decimal = deg + minutes / 60
    if direction in ['S', 'W']:
        decimal = -decimal
    return round(decimal, 6)

def get_gps_coordinates(ser):
    ser.write(b'AT+CGPSINFO\r')
    time.sleep(1)
    response = ser.read(ser.in_waiting).decode(errors='ignore')
    if "+CGPSINFO:" in response:
        try:
            data = response.split("+CGPSINFO:")[1].split("\r\n")[0].strip()
            parts = data.split(",")
            if parts[0] and parts[2]:
                lat = nmea_to_decimal(parts[0], parts[1])
                lon = nmea_to_decimal(parts[2], parts[3])
                return lat, lon
        except Exception as e:
            print("Erreur de parsing:", e)
    return None, None

def pygame_loop(get_coords, stop_event):
    pygame.init()
    screen = pygame.display.set_mode((600, 150))
    pygame.display.set_caption("SIM7600 GPS Tracker")
    font = pygame.font.SysFont("monospace", 24)

    while not stop_event.is_set():
        screen.fill((0, 0, 0))
        lat, lon = get_coords()
        if lat and lon:
            text = f"Lat: {lat}  Lon: {lon}"
        else:
            text = "Fix GPS en attente..."

        label = font.render(text, True, (0, 255, 0))
        screen.blit(label, (20, 60))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_event.set()

        time.sleep(1)

    pygame.quit()

# === PROGRAMME PRINCIPAL ===
if __name__ == "__main__":
    current_coords = [None, None]
    stop_event = threading.Event()

    def get_current_coords():
        return current_coords[0], current_coords[1]

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2) as ser:
            print("Activation du GPS...")
            ser.write(b'AT+CGPS=1,1\r')
            time.sleep(3)

            print("Fenêtre de position Pygame en cours...")
            display_thread = threading.Thread(target=pygame_loop, args=(get_current_coords, stop_event))
            display_thread.start()

            while not stop_event.is_set():
                lat, lon = get_gps_coordinates(ser)
                if lat and lon:
                    print(f"Position : Latitude={lat} | Longitude={lon}")
                    current_coords[0], current_coords[1] = lat, lon

                    maps_url = f"https://www.google.com/maps?q={lat},{lon}"
                    print(f"Lien Google Maps : {maps_url}\n")
                else:
                    print("GPS non fixé...")

                time.sleep(REFRESH_INTERVAL)

    except KeyboardInterrupt:
        print("\nArrêt manuel. Fermeture propre.")
        stop_event.set()
        display_thread.join()
        sys.exit(0)

    except serial.SerialException as e:
        print("Erreur série :", e)
