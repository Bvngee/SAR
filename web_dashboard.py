"""
Send stuff to the web dashboard.

Example:
```
import web_dashboard as wd

wd.id = "<your id>"
wd.init_grid(10)
wd.init_log()
wd.set_square(0, 0, "red")
wd.set_square(1, 1, "green")
wd.log("Hello, world!")
```
"""

import socket
import time

SERVER_ADDRESS = ("192.168.4.33", 3101)
id = ""
sock = None

def connect_web_server():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(SERVER_ADDRESS)



# ("red" | "green" | "blue" | "yellow" | "cyan" | "magenta" | "none" | "magnet" | "grey")

def set_square(x: int, y: int, color: str):
    if color not in [
        "red",
        "green",
        "blue",
        "yellow",
        "cyan",
        "magenta",
        "none",
        "magnet",
        "grey",
    ]:
        raise ValueError("Invalid color")

    message = f"set/{id}/{x}/{y}/{color}||".encode()
    sock.sendall(message)


def init_grid(grid_size: int):
    message = f"init/grid/{id}/{grid_size}/{grid_size}||".encode()
    sock.sendall(message)


def init_log():
    message = f"init/logs/{id}||".encode()
    sock.sendall(message)

def init_plot():
    message = f"init/plot/{id}||".encode()
    sock.sendall(message)


def log(msg: str):
    message = f"log/{id}/{msg}||".encode()
    sock.sendall(message)

def color_to_hex(color: str):
    if color == "red":
        return "FF0000"
    elif color == "green":
        return "00FF00"
    elif color == "blue":
        return "0000FF"
    elif color == "yellow":
        return "FFFF00"
    elif color == "cyan":
        return "00FFFF"
    elif color == "magenta":
        return "FF00FF"
    elif color == "none":
        return "FFFFFF"
    elif color == "magnet":
        return "FF00FF"
    elif color == "grey":
        return "808080"
    else:
        raise ValueError("Invalid color")

def plot(x: float, y: float, color: str):
    if color not in [
        "red",
        "green",
        "blue",
        "yellow",
        "cyan",
        "magenta",
        "none",
        "magnet",
        "grey",
    ]:
        raise ValueError("Invalid color")

    message = f"plot/{id}/{x}/{y}/{color_to_hex(color)}||".encode()
    sock.sendall(message)

# # time.sleep(1)
# id = "test"
# connect_web_server()
# init_plot()
# # time.sleep(1)
# plot(0, 0, "red")
# plot(1, 1, "green")
# plot(2, 2, "blue")
