# Maze Files

The simulator supports a few different maze file formats, as specified below.
If your format isn't supported, feel free to put up a pull request.

Note that, in order to use a maze in the simulator, it must be:

* Nonempty
* Rectangular
* Fully enclosed

Also note that official Micromouse mazes have additional requirements:

* No inaccessible locations
* Exactly three starting walls
* Only one entrance to the center
* Has a hollow center, i.e., the center peg has no walls attached to it
* Has walls attached to every peg except the center peg
* Is unsolvable by a wall-following robot

#### Map format

Example:

    o---o---o---o
    |       |   |
    o   o   o   o
    |   |       |
    o---o---o---o

* Each cell is 5 spaces wide and 3 spaces tall
* All characters besides spaces count as walls
* Walls are determined by checking the locations marked with an "x":

```
o x o
x   x
o x o
```

## Maze Wall Cell Dictionary 

Hereafter is reported the numerical dictionary corresponding to all possible configurations of walls around a cell in the maze. The dictionary is used to convert the ASCII representation of the maze into a numerical representation, which is easier to work with programmatically. 

| Decimal | View | E | N | W | S |
|---------|----------------|---|---|---|---|
| 0       | <pre>╷  ·<br>╵  ·</pre> | ✓ |   |   |   |
| 1       | <pre>╶──╴<br>·  ·</pre> |   | ✓ |   |   |
| 2       | <pre>·  ╷<br>·  ╵</pre> |   |   | ✓ |   |
| 3       | <pre>·  ·<br>╶──╴</pre> |   |   |   | ✓ |
| 4       | <pre>╷  ·<br>└──╴</pre> | ✓ |   |   | ✓ |
| 5       | <pre>·  ╷<br>╶──┘</pre> |   |   | ✓ | ✓ |
| 6       | <pre>╶──┐<br>·  ╵</pre> |   | ✓ | ✓ |   |
| 7       | <pre>┌──╴<br>╵  ·</pre> | ✓ | ✓ |   |   |
| 8       | <pre>╷  ╷<br>╵  ╵</pre> | ✓ |   | ✓ |   |
| 9       | <pre>╶──╴<br>╶──╴</pre> |   | ✓ |   | ✓ |
| 10      | <pre>╶──┐<br>╶──┘</pre> |   | ✓ | ✓ | ✓ |
| 11      | <pre>╷  ╷<br>└──┘</pre> | ✓ |   | ✓ | ✓ |
| 12      | <pre>┌──╴<br>└──╴</pre> | ✓ | ✓ |   | ✓ |
| 13      | <pre>┌──┐<br>╵  ╵</pre> | ✓ | ✓ | ✓ |   |
| 14      | <pre>┌──┐<br>└──┘</pre> | ✓ | ✓ | ✓ | ✓ |
| 15      | <pre>·  ·<br>·  ·</pre> |   |   |   |   |
