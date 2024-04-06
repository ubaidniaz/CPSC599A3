import requests
from geopy.geocoders import Nominatim
import serial
import time

def get_coordinates(city_name):
    geolocator = Nominatim(user_agent="599_UNIQUE_AGENT")
    location = geolocator.geocode(city_name)
    if location:
        return (location.latitude, location.longitude)
    else:
        print("Could not get coordinates for the city.")
        return (None, None)

def get_weather(latitude, longitude, ser):
    if latitude is None or longitude is None:
        print("Invalid coordinates.")
        return
    url = f"https://api.open-meteo.com/v1/forecast?latitude={latitude}&longitude={longitude}&current_weather=true"
    response = requests.get(url)
    if response.status_code == 200:
        data = response.json()
        temperature = data['current_weather']['temperature']
        wind_direction = data['current_weather']['winddirection']
        wind_speed = data['current_weather']['windspeed']  # Get wind speed from the API response
        
        # Print the received weather data
        print(f"Current temperature: {temperature}°C")
        print(f"Wind direction: {wind_direction}°")
        print(f"Wind speed: {wind_speed} km/h")  # Print wind speed
        
        # Prepare the data string to send temperature, wind direction, and wind speed
        data_string = f"T{temperature} D{wind_direction} W{wind_speed}\n"
        
        # Print the formatted data string to the console
        print(f"Sending to Arduino: {data_string.strip()}")
        
        # Send data to Arduino
        ser.write(data_string.encode())
    else:
        print(f"Failed to fetch data, status code: {response.status_code}")

if __name__ == "__main__":
    serialPort = 'COM7'  # Adjust to your Arduino's serial port
    baudRate = 9600  # Match this to your Arduino sketch's baud rate
    ser = serial.Serial(serialPort, baudRate, timeout=1)
    time.sleep(2)  # Wait for the serial connection to initialize
    
    while True:
        city_name = input("Enter city name (or type 'exit' to quit): ")
        if city_name.lower() == 'exit':
            ser.close()
            break
        latitude, longitude = get_coordinates(city_name)
        get_weather(latitude, longitude, ser)
