import src.api.mms_api as mms_api
import sys
import matplotlib.pyplot as plt
import numpy as np
import time

import tkinter as tk
from tkinter import ttk


def show_color_legend():
    """Display a small window explaining the meaning of each cell color."""

    legend = [
        ("Goal Area", "#008000"),          # G
        ("Reached Goal", "#00FF00"),       # g
        ("Visited once", "#0000FF"),       # b
        ("Visited twice", "#00FFFF"),      # c
        ("Visited three times", "#FFFF00"),# y
        ("Visited four times", "#FFA500"), # o
        ("Visited five times", "#FF0000"), # r
        ("Visited >5 times", "#8B0000"),   # R
    ]

    root = tk.Tk()
    root.title("Maze Color Legend")
    root.resizable(False, False)

    frame = ttk.Frame(root, padding=12)
    frame.pack(fill="both", expand=True)

    ttk.Label(
        frame,
        text="Cell Color Legend",
        font=("Arial", 12, "bold")
    ).pack(anchor="w", pady=(0, 10))

    for text, color in legend:
        row = ttk.Frame(frame)
        row.pack(anchor="w", pady=2)

        canvas = tk.Canvas(
            row,
            width=22,
            height=22,
            highlightthickness=1,
            highlightbackground="black"
        )
        canvas.create_rectangle(0, 0, 22, 22, fill=color, outline=color)
        canvas.pack(side="left")

        ttk.Label(row, text=f"  {text}", font=("Arial", 10)).pack(side="left")

    root.mainloop()

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def setOccurenceColor(x, y, count):
    if count == 0:
        return
    elif count == 1:
        mms_api.setColor(x, y, "b")
    elif count == 2:
        mms_api.setColor(x, y, "c")
    elif count == 3:
        mms_api.setColor(x, y, "y")
    elif count == 4:
        mms_api.setColor(x, y, "o")
    elif count == 5:
        mms_api.setColor(x, y, "r")
    else:
        mms_api.setColor(x, y, "R")

def getDirection(pos):
    if pos == 0:
        return "n"
    elif pos == 1:
        return "e"
    elif pos == 2:
        return "s"
    elif pos == 3:
        return "w"

def main(logger=False):
    if logger:
        log("Running...")
    mms_api.setText(0, 0, "s0")

    x = 0 ; y = 0
    goals = [(7,7), (7,8), (8,7), (8,8)] # square goal area in the center of the maze

    # Color goal area in `G` (green)
    for gx, gy in goals:
        mms_api.setColor(gx, gy, "G")

    # define np matrix 16 x 16 to represent the maze, initialize all values to 0
    maze = np.zeros((16, 16), dtype=int)

    maze[0, 0] = 1 # starting position
    setOccurenceColor(x, y, maze[y, x]) # set color of starting position

    pos = 0 # 0: up, 1: right, 2: down, 3: left
    moves = 0
    while moves < 500 and (x, y) not in goals:
        #time.sleep(0.25)
        if not mms_api.wallLeft():
            mms_api.turnLeft()
            mms_api.setWall(x, y, getDirection(pos))
            pos = (pos - 1) % 4
        while mms_api.wallFront():
            mms_api.turnRight()
            mms_api.setWall(x, y, getDirection(pos))
            pos = (pos + 1) % 4
        mms_api.moveForward()
        if pos == 0:
            y += 1
        elif pos == 1:
            x += 1
        elif pos == 2:
            y -= 1
        elif pos == 3:
            x -= 1

        maze[y, x] += 1 # count occurrences of visits to each cell
        if (x, y) not in goals:
            setOccurenceColor(x, y, maze[y, x])
        else:
            mms_api.setColor(x, y, "g") # set color of goal area to bright green

        mms_api.setText(x, y, "{x} {y}".format(x=x, y=y))
        
        if logger:
            log("x: {}, y: {}, pos: {}".format(x, y, pos))
        moves += 1
    
    # mirror the maze matrix vertically to match the simulator's coordinate system
    maze = np.flipud(maze)

    # print the maze matrix to stderr
    if logger:
        log("Maze matrix:")
        log(maze)

    # plot the maze matrix using matplotlib
    plt.imshow(maze, cmap='OrRd', interpolation='nearest')
    plt.colorbar()
    # plot the goal area in blue
    plt.scatter([7, 7, 8, 8], [7, 8, 7, 8], color='blue', s=100)
    plt.show()

import multiprocessing

# if passed as flag --logger, will log the mouse's position and orientation to stderr
if __name__ == "__main__":
    multiprocessing.Process(target=show_color_legend).start()
    main(logger="--logger" in sys.argv)