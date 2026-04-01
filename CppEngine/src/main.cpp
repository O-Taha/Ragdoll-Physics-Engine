#include <iostream>
#include <string>
#include <functional>
#include <vector>
#include "raylib.h"
#include "raymath.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

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

Point* selectedPoint = nullptr;
Point* selectedTarget = nullptr;
//bool isDragging = false;
float grabRadius = 15.0f;

enum mouseMode {
    IDLE, // NO INFLUENCE ON SCREEN
    DRAGGING, // TO DRAG POINTS ACCROSS THE SCREEN
    TARGETING, // TO TARGET A POINT TO SHOOT IT
    PLACING // TO PLACE A BODY
};

mouseMode mouseState = IDLE; 

// Globale pour GUI

bool boxEnableGround = false;
bool boxEnableGravity = true;
bool boxEnableWind = false;
bool boxEnableBullet = false;


bool btnAddBodyPressed = false;
bool btnDeleteBodyPressed = false;

bool btnAddVolumePressed = false;
bool btnDeleteVolumePressed = false;

bool btnConfigGravityPressed = false;
bool btnConfigWindPressed = false;
bool btnConfigBulletPressed = false;

bool btnConfigGravityWindow = false;
bool btnConfigWindWindow = false;
bool btnConfigBulletWindow = false;

bool btnSelectTarget = false;
bool btnShootTarget = false;

bool btnRunPressed = false;

bool textBoxGravityEditMode = false;
char textGravityInput[16] = "-9.81";

bool textBoxWindXEditMode = false;
char textWindXInput[16] = "0";

bool textBoxWindYEditMode = false;
char textWindYInput[16] = "0";

bool textBoxWindZEditMode = false;
char textWindZInput[16] = "0";

bool textBoxBulletXEditMode = false;
char textBulletXInput[16] = "0";

bool textBoxBulletYEditMode = false;
char textBulletYInput[16] = "0";

bool textBoxBulletZEditMode = false;
char textBulletZInput[16] = "0";

Vector3 impulseBuf = Vector3{0.0f, 0.0f, 0.0f};

void addBody(){}
void deleteBody(){}
void addVolume(){}
void deleteVolume(){}

void enableGround(){}
void enableGravity(){}
void enableWind(){}
void enableBullet(){}

//


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
        Point(Vector3 pos, Vector3 vel, float w = 1.0): pos(pos), vel(vel), w(w), oldW(w){
            this->recordPos.push_back(pos);
        }

        const Vector3& pos_() const     { return pos; }
        const Vector3& oldPos_() const  { return oldPos; }
        const Vector3& vel_() const     { return vel; }
        const Vector3& acc_() const     { return acc; }
        const float w_() const          { return w; }
        const float oldW_() const          { return oldW; }
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

        void impulse(const Vector3& impulseForce) {
            if(w == 0.0f) return;

            vel += impulseForce;
        }

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
        static std::vector<Body*> bodies;

        Body( // Constructor
            std::vector<Point*> points,
            std::vector<Edge*> edges,
            std::vector<std::function<Vector3(World&, Body&, Point&)>> forces,      
            bool wireframe,
            bool freeze
        ) : points(points), edges(edges), forces(forces), wireframe(wireframe), freeze(freeze) {
            bodies.push_back(this);
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
std::vector<Body*> Body::bodies;

class World {
    public:
    std::vector<Body*> bodies;
    float t;    // time
    float h;    // time-step
    float T;    // Full sim duration
    float gravity;
    Vector3 wind;

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

            Vector3 u_norm = (Vector3Length(u) < 1e-6f) ? Vector3{0,0,0} : Vector3Normalize(u);
            Vector3 v_norm = (Vector3Length(v) < 1e-6f) ? Vector3{0,0,0} : Vector3Normalize(v);

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

        World(std::vector<std::function<Vector3(World&, Body&, Point&)>> forces, std::vector<Body*> bodies, float T = 0, float h = 1.0, float gravity = -9.81, Vector3 wind = Vector3{0.0f, 0.0f, 0.0f}): globalForces(forces), bodies(bodies), t(0), T(T), h(h), gravity(gravity), wind(wind){}
        
        const float& gravity_() const     { return gravity; }
        void gravity_(const float& g)     { gravity = g; }

        const Vector3& wind_() const     { return wind; }
        void wind_(const Vector3& w)     { wind = w; }

        const float& windX_() const     { return wind.x; }
        void windX_(const float& wx)     { wind.x = wx; }

        const float& windY_() const     { return wind.y; }
        void windY_(const float& wy)     { wind.y = wy; }

        const float& windZ_() const     { return wind.z; }
        void windZ_(const float& wz)     { wind.z = wz; }

        void runStep(float dt = -1) {
            h = (dt <= 0)? h : dt; // if no time-step provided, default to class member h
            for (Body*& body : bodies) {
                if (! body->wireframe) continue; // HACK: REFACTOR (if body is volume, wall... don't apply verlet)
                
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
            if (recordTime.size() > 1000)
                recordTime.erase(recordTime.begin());

            recordTime.push_back(t);
        }
};

Point* getPointUnderCursor(Vector2 mouse)
{
    Vector3 m = screenToWorld(mouse);


    for (Body*& b : Body::bodies)
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
    if(!boxEnableGravity) return Vector3{0.0f, 0.0f, 0.0f};
    
    if (point.w_() == 0.0f)
        return Vector3{0.0f, 0.0f, 0.0f};

    return Vector3{0.0f, (world.gravity_() * 100.0f) / point.w_(), 0.0f};
};

auto wind = [](World& world, Body& body, Point& point) -> Vector3
{
    if(!boxEnableWind) return Vector3{0.0f, 0.0f, 0.0f};
    
    if (point.w_() == 0.0f)
        return Vector3{0.0f, 0.0f, 0.0f};

    return world.wind_() * 100.0f;
};

void loadPendulum()
{
    Point* p1 = new Point({400, 400, 0}, {0,0,0}, 0.0f); // fixed
    Point* p2 = new Point({350, 300, 0}, {0,0,0}, 1.0f);

    Edge* e1 = new Edge(*p1, *p2, 100.0f, 1.0f);

    Body* body = new Body({p1, p2}, {e1}, {}, true, false);

    world = new World({gravity, wind}, Body::bodies, 0.0f, 0.016f);

}

void loadBody(const Vector3& mousePos)
{
    float x = mousePos.x, y = mousePos.y;

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

    world->bodies.push_back(body);
}

void loadGround()
{
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

    world = new World({gravity, wind}, Body::bodies, 0.0f, 0.016f);
}

// ----------------------
// SCENE SWITCH
// ----------------------

void deleteAllBodies(){
    for (auto b : Body::bodies) {
        for (auto p : b->points) delete p;
        for (auto e : b->edges) delete e;
        delete b;
    }
    Body::bodies.clear();
}

void loadScene(int scene)
{
    selectedPoint = nullptr;
    //isDragging = false;
    mouseState = IDLE;
    
    if (world != nullptr) {
        delete world;
        world = nullptr;
    }
    deleteAllBodies();

    switch(scene)
    {
        case PENDULUM: loadPendulum(); break;
        case RAGDOLL:  loadGround();  break;
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
        dt = fminf(dt, 0.016f);

        if (running) {
            Vector2 mouse = GetMousePosition();

                // -------- INPUT --------
            if (IsKeyPressed(KEY_RIGHT)) {
                //isDragging = false;
                mouseState = IDLE;
                selectedPoint = nullptr;
                currentScene = (currentScene + 1) % SCENE_COUNT;
                loadScene(currentScene);
            }

            if (IsKeyPressed(KEY_LEFT)) {
                //isDragging = false;
                mouseState = IDLE;
                selectedPoint = nullptr;
                currentScene = (currentScene - 1 + SCENE_COUNT) % SCENE_COUNT;
                loadScene(currentScene);
            }

            //if (IsKeyPressed(KEY_K)) {
            //    std::cout << "impulse" << std::endl;
            //    world->bodies[0]->points[1]->impulse(Vector3{500, 0, 0});
            //}

            // CLICK
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
            {
                std::cout << mouseState << std::endl;
                selectedPoint = getPointUnderCursor(mouse);
                if (mouseState == PLACING){
                        loadBody(screenToWorld(mouse));
                        std::cout << "Placing body" << std::endl;
                        mouseState = IDLE;
                }
                else if (selectedPoint)
                {
                    //isDragging = true;
                    if(mouseState == IDLE){
                        mouseState = DRAGGING;
                        selectedPoint->invW_(0.0f); // fixe temporairement
                        std::cout << "Dragging point" << std::endl;
                    }
                    else if (mouseState == TARGETING){
                        selectedTarget = selectedPoint;
                        std::cout << selectedPoint->pos_().x << selectedPoint->pos_().y << std::endl;
                        mouseState = IDLE;
                        std::cout << "Targetting point" << std::endl;
                    }
                }
            }

            // RELEASE
            if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
            {
                if (selectedPoint)
                {
                    selectedPoint->invW_(selectedPoint->oldW_());
                }
                selectedPoint = nullptr;
                //isDragging = false;
                if(mouseState != TARGETING && mouseState != PLACING) mouseState = IDLE;
            }

            // DRAG
            if (mouseState == DRAGGING && selectedPoint)
            {
                Vector3 m = screenToWorld(mouse);

                selectedPoint->pos_(m);
                selectedPoint->oldPos_(m); // CRUCIAL sinon explosion physique
                selectedPoint->vel_({0,0,0});
            }

            // -------- UPDATE --------
            if (world) world->runStep(dt);

        }
        // -----
      
        if(boxEnableGround) enableGround();
        if(boxEnableGravity) enableGravity();
        if(boxEnableWind) enableWind();
        if(boxEnableBullet) enableBullet();
      
        // -------- DRAW --------
    
        BeginDrawing();
        ClearBackground(RAYWHITE);
      
            DrawLine(500, 0, 500, GetScreenHeight(), Fade(LIGHTGRAY, 0.6f));
            DrawRectangle(500, 0, GetScreenWidth() - 500, GetScreenHeight(), Fade(LIGHTGRAY, 0.3f));

            GuiSetStyle(DEFAULT, TEXT_SIZE, 24);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
            GuiSetStyle(LABEL, TEXT_PADDING, 5);
            GuiSetStyle(LABEL, TEXT_COLOR_NORMAL, ColorToInt(GRAY));
            GuiLabel((Rectangle){500, 10, GetScreenWidth() - 500, 32}, "Settings");
            
            GuiSetStyle(DEFAULT, TEXT_SIZE, 16);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
            GuiSetStyle(LABEL, TEXT_PADDING, 5);
            GuiSetStyle(LABEL, TEXT_COLOR_NORMAL, ColorToInt(GRAY));

            GuiLabel((Rectangle){500, 30, GetScreenWidth() - 500, 32}, "Bodies & shapes");

            GuiLabel((Rectangle){500, 170, GetScreenWidth() - 500, 32}, "World Environment");

            GuiLabel((Rectangle){500, 310, GetScreenWidth() - 500, 32}, "Bullet");

            GuiCheckBox((Rectangle){505, 140, 20, 20}, "Enable Ground", &boxEnableGround);
            GuiCheckBox((Rectangle){640, 200, 20, 20}, "Enable Gravity", &boxEnableGravity);
            GuiCheckBox((Rectangle){640, 240, 20, 20}, "Enable Wind", &boxEnableWind);
            GuiCheckBox((Rectangle){640, 340, 20, 20}, "Enable Bullet", &boxEnableBullet);

            GuiSetStyle(DEFAULT, TEXT_SIZE, 12);

            if(GuiButton((Rectangle){505, 60, 120, 30}, "#149#Add Body")) btnAddBodyPressed = !btnAddBodyPressed;

            if(btnAddBodyPressed)
            {
                mouseState = PLACING;
                btnAddBodyPressed = false;
                std::cout << "MODE PLACING" << std::endl;
            }

            if(GuiButton((Rectangle){640, 60, 120, 30}, "#143#Del. Body")) deleteBody();

            if(GuiButton((Rectangle){505, 100, 120, 30}, "#162#Add Volume")) addVolume();
            if(GuiButton((Rectangle){640, 100, 120, 30}, "#143#Del. Body")) deleteVolume();
            
            if(GuiButton((Rectangle){505, 200, 120, 30}, "#142#Config Gravity")) btnConfigGravityWindow = !btnConfigGravityWindow;

            if(btnConfigGravityWindow)
            {
                DrawRectangle(150, 100, 200, 110, Fade(LIGHTGRAY, 0.3f));
                if (GuiTextBox((Rectangle){ 40, 64, 720, 32 }, textGravityInput, 95, textBoxGravityEditMode))
                {
                    textBoxGravityEditMode = !textBoxGravityEditMode;
                    
                    world->gravity_(std::stof(textGravityInput));
                    std::cout << world->gravity_() << std::endl;
                }
            }

            if(GuiButton((Rectangle){505, 240, 120, 30}, "#142#Config Wind")) btnConfigWindWindow = !btnConfigWindWindow;

            if(btnConfigWindWindow)
            {
                DrawRectangle(150, 100, 200, 110, Fade(LIGHTGRAY, 0.3f));
                if (GuiTextBox((Rectangle){ 40, 64, 720, 32 }, textWindXInput, 95, textBoxWindXEditMode))
                {
                    textBoxWindXEditMode = !textBoxWindXEditMode;
                    
                    world->windX_(std::stof(textWindXInput));
                    std::cout << world->windX_() << std::endl;
                }
                if (GuiTextBox((Rectangle){ 40, 100, 720, 32 }, textWindYInput, 95, textBoxWindYEditMode))
                {
                    textBoxWindYEditMode = !textBoxWindYEditMode;
                    
                    world->windY_(std::stof(textWindYInput));
                    std::cout << world->windY_() << std::endl;
                }
                if (GuiTextBox((Rectangle){ 40, 160, 720, 32 }, textWindZInput, 95, textBoxWindZEditMode))
                {
                    textBoxWindZEditMode = !textBoxWindZEditMode;
                    
                    world->windZ_(std::stof(textWindZInput));
                    std::cout << world->windZ_() << std::endl;
                }
            }

            if(GuiButton((Rectangle){505, 340, 120, 30}, "#142#Config Bullet")) btnConfigBulletWindow = !btnConfigBulletWindow;

            if(btnConfigBulletWindow)
            {
                DrawRectangle(150, 100, 200, 110, Fade(LIGHTGRAY, 0.3f));
                // Selection du point
                if(GuiButton((Rectangle){150, 100, 120, 30}, "#142#Select Target")) btnSelectTarget = !btnSelectTarget;

                    if(btnSelectTarget)
                    {
                        mouseState = TARGETING;
                        std::cout << "MODE TARGET" << std::endl;

                        btnSelectTarget = false;

                    }

                if(GuiValueBoxFloat((Rectangle){150, 200, 100, 20}, "x", textBulletXInput, &impulseBuf.x, textBoxBulletXEditMode)) textBoxBulletXEditMode = !textBoxBulletXEditMode;
                if(GuiValueBoxFloat((Rectangle){150, 250, 100, 20}, "y", textBulletYInput, &impulseBuf.y, textBoxBulletYEditMode)) textBoxBulletYEditMode = !textBoxBulletYEditMode;
                if(GuiValueBoxFloat((Rectangle){150, 300, 100, 20}, "z", textBulletZInput, &impulseBuf.z, textBoxBulletZEditMode)) textBoxBulletZEditMode = !textBoxBulletZEditMode;
                
                if(GuiButton((Rectangle){150, 150, 120, 30}, "#142#Shoot Target")) btnShootTarget = !btnShootTarget;

                    if(btnShootTarget)
                    {
                        if(selectedTarget) selectedTarget->impulse(impulseBuf);

                        btnShootTarget = false;
                    }
                
            }
            
            if(GuiButton((Rectangle){505, 400, 120, 30}, "#134#Run")) running = !running;
      
        // -------

            for (auto b : Body::bodies) b->Draw();
            
            if(selectedTarget) selectedTarget->Draw(GREEN);

            DrawText(TextFormat("Scene: %d", currentScene), 10, 10, 20, BLACK);
            

        EndDrawing();
    }

    CloseWindow();
    
    return 0;
}


/*
TO DO

Convertir les boutons qu'il faut en bouton toggle
Text box en value box
Tester les limites du bouton run 
Mettre en forme
Delete Body

Add volume

Delete Volume


*/