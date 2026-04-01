## Body Class

### Construction

- Create a body from points and edges
```cpp
Body body(points, edges, forces, wireframe, freeze);
````

---

### Usage

* Group multiple points and edges into a single structure

```cpp id="r9c2mf"
std::vector<Point*> points = {&p1, &p2, &p3, &p4};
std::vector<Edge*> edges = {&e1, &e2, &e3, &e4};
Body body(points, edges, forces, wireframe, freeze);
```

---

### Physics / Update

* Attach custom forces to the body

```cpp id="k5x8qp"
std::vector<std::function<Vector3(World&, Body&, Point&)>> forces;
```

* Choose whether the body is a filled volume or a mesh

```cpp id="z1w6lf"
body.wireframe;
```

* Toggle physics applied to body
```cpp
body.freeze;
```
---

### Display

* Draw all points and edges of the body

```cpp id="n3v7ds"
body.Draw();
```



