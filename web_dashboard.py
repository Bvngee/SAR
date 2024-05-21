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


SERVER_ADDRESS = ("192.168.4.33", 3101)
#SERVER_ADDRESS = ("172.20.10.15", 3101) # .3 .15 
id = ""
sock: socket.socket = None

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


def log(msg: str):
    message = f"log/{id}/{msg}||".encode()
    sock.sendall(message)
