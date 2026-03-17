## Edge Class

### Construction

- Create an edge between two points
```cpp
Edge e(p1, p2, length = Vector2Distance(p1.pos_(), p2.pos_()), stiffness = 1);
```

---

### Display

* Draw the edge between the two points

```cpp
e.Draw();
```

