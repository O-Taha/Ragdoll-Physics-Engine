#include <iostream>
#include <functional>
#include <vector>
#include "raylib.h"
#include "raymath.h"

/*===========[ How to run ]===========
cd CppEngine/              ;
#[Wait for container to launch...]
make docker-run           
/====================================*/

#define CORRECTION_PASS 10 // Number of iterations for collision detection

bool running = false;

class World;
class Body;
class Point;
class Edge;

enum SceneType {
    PENDULUM = 0,
    RAGDOLL,
    SCENE_COUNT
};

int currentScene = 0;
World* world = nullptr;
std::vector<Body*> bodies;

Point* selectedPoint = nullptr;
bool isDragging = false;
float grabRadius = 15.0f;

inline Vector2 toVector2(const Vector3& v) {return {v.x, v.y};}
inline Vector2 worldToScreen(const Vector3& v) {return {v.x, (float)GetScreenHeight() - v.y};}
Vector3 screenToWorld(Vector2 mouse)
{return Vector3{
        mouse.x,
        (float)GetScreenHeight() - mouse.y,
        0
        };
}

class Point {
    private:
        Vector3 pos;
        Vector3 oldPos;
        Vector3 vel;
        Vector3 acc;
        float w{1.0f}; // /!\ Inverse weight
        float oldW;
        std::vector<Vector3> recordPos;
        Body* owner;

    public:
        Point(Vector3 pos, Vector3 vel, float w = 1.0): pos(pos), vel(vel), w(w){
            this->recordPos.push_back(pos);
        }

        const Vector3& pos_() const     { return pos; }
        const Vector3& oldPos_() const  { return oldPos; }
        const Vector3& vel_() const     { return vel; }
        const Vector3& acc_() const     { return acc; }
        const float w_() const          { return w; }
        const Body* owner_() const      { return owner; }

        void pos_(const Vector3& v)     { pos = v; }
        void oldPos_(const Vector3& v)  { oldPos = v; }
        void incrementPos(float dx, float dy = 0) { 
            pos.x += dx; pos.y += dy;
        }
        void vel_(const Vector3& v)     { vel = v; }
        void acc_(const Vector3& v)     { acc = v; }
        void w_(float v)                { w = 1/v; }
        void invW_(float v)             { w = v; }
        void owner_(Body* b)            { owner = b; }

        void Draw(Color c = RED) {DrawCircleV(worldToScreen(pos), 5, c);}
};

class Edge {
    public:
        Point& p1;
        Point& p2;
        float len;      // Resting length
        float stiff;    // Stiffness

        Edge(Point& p1, Point& p2, float l = -1, float s = 1): p1(p1), p2(p2), len(l), stiff(s) {
            if (l == -1) this -> len = Vector2Distance(toVector2(p1.pos_()), toVector2(p2.pos_())); // if no resting length is provided, compute it based on distance between points
        }
        void Draw(Color c = RED) {DrawLineV(worldToScreen(p1.pos_()), worldToScreen(p2.pos_()), c);}
};

class Body {
    public:
        std::vector<Point*> points;
        std::vector<Edge*> edges;
        std::vector<std::function<Vector3(World&, Body&, Point&)>> forces;
        bool wireframe;
        bool freeze;

        Body( // Constructor
            std::vector<Point*> points,
            std::vector<Edge*> edges,
            std::vector<std::function<Vector3(World&, Body&, Point&)>> forces,      
            bool wireframe,
            bool freeze
        ) : points(points), edges(edges), forces(forces), wireframe(wireframe), freeze(freeze) {
            for (Point*& p : this->points) {
                p->owner_(this);
            }
        }

        void Draw() {
            for (auto& point : this->points) {
                point->Draw(BLUE);
            }
            for (auto& edge : this->edges) {
                edge->Draw(RED);
            }
        }
};

class World {
    std::vector<Body*> bodies;
    float t;    // time
    float h;    // time-step
    float T;    // Full sim duration
    std::vector<float> recordTime;

    void applyVerlet(Point& p) {
        Body* body = const_cast<Body*>(p.owner_());

        Vector3 acc_n = computeAccel(*body, p);
        p.acc_(acc_n);

        Vector3 newPos =
            Vector3Add(
                Vector3Add(
                    p.pos_(),
                    Vector3Scale(p.vel_(), h)
                ),
                Vector3Scale(p.acc_(), (h*h)/2.0f)
            );

        p.pos_(newPos);

        p.acc_(computeAccel(*body, p));

        Vector3 newVel =
            Vector3Add(
                p.vel_(),
                Vector3Scale(Vector3Add(acc_n, p.acc_()), h/2.0f)
            );

        p.vel_(newVel);
    }

    Vector3 computeAccel(Body& body, Point& point) {
        Vector3 total_force = {0,0,0};

        for (auto& f : globalForces)
            total_force = Vector3Add(total_force, f(*this, body, point));

        for (auto& f : body.forces)
            total_force = Vector3Add(total_force, f(*this, body, point));

        return Vector3Scale(total_force, point.w_());
    }

    void applyConstraints(Edge& edge) {
        Point& p1 = edge.p1;
        Point& p2 = edge.p2;

        float w1 = p1.w_();
        float w2 = p2.w_();

        if ((w1 + w2) == 0.0f) return;

        Vector3 d = Vector3Subtract(p2.pos_(), p1.pos_());
        float len = Vector3Length(d);

        if (len < 0.0001f) return;

        float err = (len - edge.len) / len;

        Vector3 correction = Vector3Scale(d, err * edge.stiff);

        p1.pos_(Vector3Add(
            p1.pos_(),
            Vector3Scale(correction, w1 / (w1 + w2))
        ));

        p2.pos_(Vector3Subtract(
            p2.pos_(),
            Vector3Scale(correction, w2 / (w1 + w2))
        ));
    }

    void checkCollisionBetween(Body& volume, Point& p) {

        if (volume.points.size() < 4) return;

        Vector3 A = volume.points[0]->pos_();
        Vector3 B = volume.points[1]->pos_();
        Vector3 D = volume.points[3]->pos_();

        Vector3 u = Vector3Subtract(B, A);
        Vector3 v = Vector3Subtract(D, A);
        Vector3 ap = Vector3Subtract(p.pos_(), A);

        float u_len_sq = Vector3DotProduct(u, u);
        float v_len_sq = Vector3DotProduct(v, v);

        if (u_len_sq == 0 || v_len_sq == 0) return;

        float mu = Vector3DotProduct(ap, u) / u_len_sq;
        float mv = Vector3DotProduct(ap, v) / v_len_sq;

        if (mu >= 0 && mu <= 1 && mv >= 0 && mv <= 1) {

            Vector3 old_pos_before = p.pos_();

            float dist_left   = mu * Vector3Length(u);
            float dist_right  = (1 - mu) * Vector3Length(u);
            float dist_bottom = mv * Vector3Length(v);
            float dist_top    = (1 - mv) * Vector3Length(v);

            float min_d = std::min(std::min(dist_left, dist_right),
                                std::min(dist_bottom, dist_top));

            Vector3 u_norm = Vector3Normalize(u);
            Vector3 v_norm = Vector3Normalize(v);

            if (min_d == dist_left)
                p.pos_(Vector3Subtract(p.pos_(), Vector3Scale(u_norm, dist_left)));

            else if (min_d == dist_right)
                p.pos_(Vector3Add(p.pos_(), Vector3Scale(u_norm, dist_right)));

            else if (min_d == dist_bottom)
                p.pos_(Vector3Subtract(p.pos_(), Vector3Scale(v_norm, dist_bottom)));

            else
                p.pos_(Vector3Add(p.pos_(), Vector3Scale(v_norm, dist_top)));

            // friction (Verlet style)
            Vector3 velocity = Vector3Subtract(p.pos_(), p.oldPos_());
            p.oldPos_(Vector3Add(p.oldPos_(), Vector3Scale(velocity, (1 - 0.1f))));
        }
    }

    public:
        std::vector<std::function<Vector3(World&, Body&, Point&)>> globalForces;

        World(std::vector<std::function<Vector3(World&, Body&, Point&)>> forces, std::vector<Body*> bodies, float T = 0, float h = 1.0): globalForces(forces), bodies(bodies), t(0), T(T), h(h){}

        void runStep(float dt = -1) {
            h = (dt <= 0)? h : dt; // if no time-step provided, default to class member h
            for (Body*& body : bodies) {
                if (! body->wireframe) continue; // HACK: REFACTOR
                
                if (! body->freeze) {
                    for (Point*& p : body->points) {
                        p->oldPos_(Vector3(p->pos_())); // Save position to recompute speed after constraints, collisions
                        applyVerlet(*p);
                    }
                }

                for (int i = 0; i < CORRECTION_PASS; i++) {
                    for (Point*& p : body->points) { // FIX: Pairwise collision (inefficient)
                        for (Body*& potentialCollidingBody : bodies) {
                            if ((potentialCollidingBody != /*current*/body) 
                            && (! potentialCollidingBody->wireframe)) 
                            checkCollisionBetween(*potentialCollidingBody, *p);
                        }
                    }

                    for (Edge*& e : body->edges) applyConstraints(*e);
                }

                for (Point*& p : body->points) p->vel_((p->pos_() - p->oldPos_())/h);
            }
            
            t += h;
            recordTime.push_back(t);
        }
};

Point* getPointUnderCursor(Vector2 mouse)
{
    Vector3 m = screenToWorld(mouse);

    for (Body*& b : bodies)
    {
        for (Point*& p : b->points)
        {
            if (Vector3Distance(p->pos_(), m) < grabRadius)
                return p;
        }
    }
    return nullptr;
}

auto gravity = [](World& world, Body& body, Point& point) -> Vector3
{
    if (point.w_() == 0.0f)
        return Vector3{0.0f, 0.0f, 0.0f};

    return Vector3{0.0f, (-9.81f * 100.0f) / point.w_(), 0.0f};
};

void loadPendulum()
{
    for (auto b : bodies) {
        for (auto p : b->points) delete p;
        for (auto e : b->edges) delete e;
        delete b;
    }
    bodies.clear();

    Point* p1 = new Point({400, 400, 0}, {0,0,0}, 0.0f); // fixed
    Point* p2 = new Point({350, 300, 0}, {0,0,0}, 1.0f);

    Edge* e1 = new Edge(*p1, *p2, 100.0f, 1.0f);

    Body* body = new Body({p1, p2}, {e1}, {}, true, false);
    body->forces.push_back(gravity);

    bodies.push_back(body);

    world = new World({gravity}, bodies, 0.0f, 0.016f);
}

void loadRagdoll()
{
    for (auto b : bodies) {
        for (auto p : b->points) delete p;
        for (auto e : b->edges) delete e;
        delete b;
    }
    bodies.clear();

    float x = 400, y = 350;

    Point* head = new Point({x, y, 0}, {0,0,0}, 1.0f);
    Point* torso = new Point({x, y-60, 0}, {0,0,0}, 1.0f);
    Point* pelvis = new Point({x, y-120, 0}, {0,0,0}, 1.0f);

    Point* l_arm = new Point({x-80, y-60, 0}, {0,0,0}, 1.0f);
    Point* r_arm = new Point({x+80, y-60, 0}, {0,0,0}, 1.0f);

    Point* l_leg = new Point({x-20, y-200, 0}, {0,0,0}, 1.0f);
    Point* r_leg = new Point({x+20, y-200, 0}, {0,0,0}, 1.0f);

    auto makeEdge = [](Point* a, Point* b)
    {
        return new Edge(*a, *b, Vector3Distance(a->pos_(), b->pos_()), 1.0f);
    };

    std::vector<Point*> pts = {
        head, torso, pelvis, l_arm, r_arm, l_leg, r_leg
    };

    std::vector<Edge*> edges = {
        makeEdge(head, torso),
        makeEdge(torso, pelvis),
        makeEdge(torso, l_arm),
        makeEdge(torso, r_arm),
        makeEdge(pelvis, l_leg),
        makeEdge(pelvis, r_leg)
    };

    Body* body = new Body(pts, edges, {}, true, false);
    body->forces.push_back(gravity);

    bodies.push_back(body);

    // ----- SOL -----
    Point* s1 = new Point({0, 0, 0}, {0,0,0}, 0.0f);
    Point* s2 = new Point({800, 0, 0}, {0,0,0}, 0.0f);
    Point* s3 = new Point({800, 50, 0}, {0,0,0}, 0.0f);
    Point* s4 = new Point({0, 50, 0}, {0,0,0}, 0.0f);

    Edge* se1 = new Edge(*s1, *s2);
    Edge* se2 = new Edge(*s2, *s3);
    Edge* se3 = new Edge(*s3, *s4);
    Edge* se4 = new Edge(*s4, *s1);

    Body* ground = new Body(
        {s1, s2, s3, s4},
        {se1, se2, se3, se4},
        {},
        false,  // ⚠️ IMPORTANT
        true    // freeze
    );

    bodies.push_back(ground);

    world = new World({gravity}, bodies, 0.0f, 0.016f);
}

// ----------------------
// SCENE SWITCH
// ----------------------

void loadScene(int scene)
{
    if (world) delete world;

    switch(scene)
    {
        case PENDULUM: loadPendulum(); break;
        case RAGDOLL:  loadRagdoll();  break;
    }
}   

int main()
{
    InitWindow(800, 450, "Physics Engine");
    SetTargetFPS(60);

    loadScene(currentScene);
    running = true;
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.016f;

        if (IsKeyPressed(KEY_SPACE)) {
            running = !running;
        }

        if (running) {
            Vector2 mouse = GetMousePosition();

                // -------- INPUT --------
            if (IsKeyPressed(KEY_RIGHT)) {
                currentScene = (currentScene + 1) % SCENE_COUNT;
                loadScene(currentScene);
            }

            if (IsKeyPressed(KEY_LEFT)) {
                currentScene = (currentScene - 1 + SCENE_COUNT) % SCENE_COUNT;
                loadScene(currentScene);
            }

            // CLICK
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
            {
                selectedPoint = getPointUnderCursor(mouse);

                if (selectedPoint)
                {
                    isDragging = true;
                    selectedPoint->invW_(0.0f); // fixe temporairement
                }
            }

            // RELEASE
            if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
            {
                if (selectedPoint)
                {
                    selectedPoint->invW_(1.0f); // ⚠️ simplifié (tu peux stocker oldW si tu veux propre)
                    selectedPoint = nullptr;
                }
                isDragging = false;
            }

            // DRAG
            if (isDragging && selectedPoint)
            {
                Vector3 m = screenToWorld(mouse);

                selectedPoint->pos_(m);
                selectedPoint->oldPos_(m); // CRUCIAL sinon explosion physique
            }

            // -------- UPDATE --------
            world->runStep(dt);

        }
        // -------- DRAW --------
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (auto b : bodies)
            b->Draw();

        DrawText(TextFormat("Scene: %d", currentScene), 10, 10, 20, BLACK);

        EndDrawing();
    }

    CloseWindow();
}