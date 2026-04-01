## Point Class

### Construction

- Create a point with position and velocity
```cpp
Point p({x, y}, {vx, vy});
````

---

### Usage

* Increment the position using offsets

```cpp
p.incrementPos(dx, dy = 0);
```

---

### Display

* Draw the point on screen

```cpp
p.Draw(color);
```

---

### Debug

* Display the current position as text

```cpp
DrawText(
    TextFormat("(%.2f, %.2f)", p.pos_().x, p.pos_().y),
    p.pos_().x,
    p.pos_().y,
    fontSize,
    color
);
```

---

### Accessors

* Access current position

```cpp
p.pos_().x;
p.pos_().y;
```
