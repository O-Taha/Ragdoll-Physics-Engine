#include <iostream>
#include <ostream>
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
#define WINDOW_HEIGHT 720
#define WINDOW_WIDTH 1920

#define NB_DIAPO 29

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

bool pendulumLoaded = false;
Body* pendulumPointer = nullptr; // Pour pouvoir le supprimer précisément

bool springLoaded = false;
Body* springPointer = nullptr; // Pour pouvoir le supprimer précisément

bool ballAndWallLoaded = false;
Body* cube1Pointer = nullptr;
Body* cube2Pointer = nullptr;
Body* ballPointer = nullptr;

bool finalLoaded = false;
Body* groundBody = nullptr;
Body* finalWall = nullptr;
Body* ragdollBody = nullptr;

Body* diapoHanger = nullptr;


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

void enableGravity(){}
void enableWind(){}
void enableBullet(){}

static void removeFromVector(std::vector<Body*>& vec, Body* b)
{
    vec.erase(std::remove(vec.begin(), vec.end(), b), vec.end());
}

static void addToVector(std::vector<Body*>& vec, Body* b)
{
    if (std::find(vec.begin(), vec.end(), b) == vec.end())
        vec.push_back(b);
}

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

        void pos_(const Vector3& v){
            pos = v;
            // if (Vector2Length(toVector2(pos)) > WINDOW_HEIGHT * 2) delete owner; //HACK : FIX OOB Deletion
        }
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
        bool enabled;
        static std::vector<Body*> bodies;

        Body( // Constructor
            std::vector<Point*> points,
            std::vector<Edge*> edges,
            std::vector<std::function<Vector3(World&, Body&, Point&)>> forces,      
            bool wireframe,
            bool freeze
        ) : points(points), edges(edges), forces(forces), wireframe(wireframe), freeze(freeze), enabled(true) {
            bodies.push_back(this);
            for (Point*& p : this->points) {
                p->owner_(this);
            }
        }

        ~Body(){
            for (Point* p : points) delete p;
            for (Edge* e : edges) delete e;
        }

        void disable(std::vector<Body*>& body_vec)
        {
            if (!enabled) return;

            enabled = false;

            // Retirer des collisions (World)
            removeFromVector(body_vec, this);

            // Optionnel : retirer aussi du global
            removeFromVector(Body::bodies, this);
        }

        void enable(std::vector<Body*>& body_vec)
        {
            if (enabled) return;

            enabled = true;

            // Réintégrer dans les collisions
            addToVector(body_vec, this);

            // Réintégrer dans le global
            addToVector(Body::bodies, this);
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
        
        float& gravity_()   { return gravity; }
        void gravity_(const float& g)     { gravity = g; }

        const Vector3& wind_() const     { return wind; }
        void wind_(const Vector3& w)     { wind = w; }

        float& windX_()     { return wind.x; }
        void windX_(const float& wx)     { wind.x = wx; }

        float& windY_()     { return wind.y; }
        void windY_(const float& wy)     { wind.y = wy; }

        float& windZ_()     { return wind.z; }
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
    Point* p1 = new Point({GetScreenWidth()/2, 400, 0}, {0,0,0}, 0.0f); // fixed
    Point* p2 = new Point({GetScreenWidth()/2, 300, 0}, {0,0,0}, 1.0f);
    Point* p3 = new Point({GetScreenWidth()/2, 200, 0}, {200,0,0}, 1.0f);

    Edge* e1 = new Edge(*p1, *p2, 100.0f, 1.0f);
    Edge* e2 = new Edge(*p2, *p3, 100.0f, 1.0f);

    pendulumPointer = new Body({p1, p2, p3}, {e1, e2}, {}, true, false);

    world->bodies.push_back(pendulumPointer);
    //world = new World({gravity, wind}, Body::bodies, 0.0f, 0.016f);

}

void loadSpring()
{
    Point* p1 = new Point({GetScreenWidth()/2, 400, 0}, {0,0,0}, 0.0f); // fixed
    Point* p2 = new Point({GetScreenWidth()/2, 300, 0}, {0,0,0}, 1.0f);

    Edge* e1 = new Edge(*p1, *p2, 100.0f, 0.01f);

    springPointer = new Body({p1, p2}, {e1}, {}, true, false);

    world->bodies.push_back(springPointer);
    //world = new World({gravity, wind}, Body::bodies, 0.0f, 0.016f);

}


void loadClothesline() {
    Point* p1 = new Point({GetScreenWidth()/2, 0, 0}, {0,0,0}, 0.0f); // fixed
    Point* p2 = new Point({GetScreenWidth()/2, -50, 0}, {0,0,0}, 1.0f);

    Edge* e1 = new Edge(*p1, *p2, 50, 1.0f);

    // On crée un seul Body qui contient toute cette structure
    diapoHanger = new Body({p1, p2}, {e1}, {}, true, false);
    world->bodies.push_back(diapoHanger);
    // Initialisation du monde si nécessaire
    //if (world == nullptr) {
    //    world = new World({gravity, wind}, Body::bodies, 0.0f, 0.016f);
    //}
}

void loadBallAndWall()
{
    // --- CUBE 1 INCLINÉ (Le toboggan) ---
    // Position du centre du cube et sa taille
    Vector3 center1 = {GetScreenWidth()/2 + 200, 350, 0 };
    float size1 = 150.0f;
    float halfSize1 = size1 / 2.0f;

    // Angle de rotation (en radians). Ex: -30 degrés pour pencher vers la droite
    float angle1 = -30.0f * DEG2RAD; 
    float cosA = cosf(angle1);
    float sinA = sinf(angle1);

    // Fonction locale pour calculer la position d'un coin après rotation
    auto calcRotatedPoint = [&](float localX, float localY) -> Vector3 {
        return {
            center1.x + (localX * cosA - localY * sinA),
            center1.y + (localX * sinA + localY * cosA),
            0
        };
    };

    // Calcul des 4 coins (toujours dans l'ordre BG, BD, HD, HG pour checkCollisionBetween)
    Point* c1_1 = new Point(calcRotatedPoint(-halfSize1, -halfSize1), {0,0,0}, 0.0f); // Bas Gauche
    Point* c1_2 = new Point(calcRotatedPoint( halfSize1, -halfSize1), {0,0,0}, 0.0f); // Bas Droite
    Point* c1_3 = new Point(calcRotatedPoint( halfSize1,  halfSize1), {0,0,0}, 0.0f); // Haut Droite
    Point* c1_4 = new Point(calcRotatedPoint(-halfSize1,  halfSize1), {0,0,0}, 0.0f); // Haut Gauche

    Edge* ec1_1 = new Edge(*c1_1, *c1_2);
    Edge* ec1_2 = new Edge(*c1_2, *c1_3);
    Edge* ec1_3 = new Edge(*c1_3, *c1_4);
    Edge* ec1_4 = new Edge(*c1_4, *c1_1);
    // Diagonale pour la rigidité structurelle (sinon le cube s'écrase)
    //Edge* ec1_d = new Edge(*c1_1, *c1_3); 

    cube1Pointer = new Body({c1_1, c1_2, c1_3, c1_4}, {ec1_1, ec1_2, ec1_3, ec1_4}, {}, false, true);

    // --- CUBE 2 BAS ET LARGE (La réception) ---
    // On le place plus bas et on le fait plus large pour bien réceptionner la balle
    float groundY = 100.0f; // Juste au dessus du sol
    float recWidth = 550.0f;
    float recHeight = 100.0f;
    float recStartX = 250.0f; // Positionné pour intercepter la trajectoire après le toboggan

    Point* c2_1 = new Point({recStartX,            groundY,             0}, {0,0,0}, 0.0f); // BG
    Point* c2_2 = new Point({recStartX + recWidth, groundY,             0}, {0,0,0}, 0.0f); // BD
    Point* c2_3 = new Point({recStartX + recWidth, groundY + recHeight, 0}, {0,0,0}, 0.0f); // HD
    Point* c2_4 = new Point({recStartX,            groundY + recHeight, 0}, {0,0,0}, 0.0f); // HG

    Edge* ec2_1 = new Edge(*c2_1, *c2_2);
    Edge* ec2_2 = new Edge(*c2_2, *c2_3);
    Edge* ec2_3 = new Edge(*c2_3, *c2_4);
    Edge* ec2_4 = new Edge(*c2_4, *c2_1);
    //Edge* ec2_d = new Edge(*c2_1, *c2_3); // Diagonale de rigidité

    cube2Pointer = new Body({c2_1, c2_2, c2_3, c2_4}, {ec2_1, ec2_2, ec2_3, ec2_4}, {}, false, true);

    // --- LA BALLE (Point dynamique) ---
    // On la place au dessus du premier cube penché pour qu'elle roule dessus
    Point* ballPoint = new Point({center1.x - 50, center1.y + size1 + 50, 0}, {0, 0, 0}, 1.0f);
    
    Point* c3_1 = new Point({GetScreenWidth()/2 - 50, 500, 0}, {0,0,0}, 1.0f); // Bas Gauche
    Point* c3_2 = new Point({GetScreenWidth()/2 + 50, 500, 0}, {0,0,0}, 1.0f); // Bas Droite
    Point* c3_3 = new Point({GetScreenWidth()/2 + 50, 550, 0}, {0,0,0}, 1.0f); // Haut Droite
    Point* c3_4 = new Point({GetScreenWidth()/2 - 50, 550, 0}, {0,0,0}, 1.0f); // Haut Gauche

    Edge* ec3_1 = new Edge(*c3_1, *c3_2);
    Edge* ec3_2 = new Edge(*c3_2, *c3_3);
    Edge* ec3_3 = new Edge(*c3_3, *c3_4);
    Edge* ec3_4 = new Edge(*c3_4, *c3_1);
    // Diagonale pour la rigidité structurelle (sinon le cube s'écrase)
    Edge* ec3_d = new Edge(*c3_1, *c3_3);

    // Wireframe = true pour qu'elle subisse la physique mais ne bloque pas les autres
    ballPointer = new Body({c3_1, c3_2, c3_3, c3_4}, {ec3_1, ec3_2, ec3_3, ec3_4, ec3_d}, {}, true, false);

    // Ajout explicite au monde (car tes constructeurs ne le font pas automatiquement dans `world->bodies`)
    world->bodies.push_back(cube1Pointer);
    world->bodies.push_back(cube2Pointer);
    world->bodies.push_back(ballPointer);
}


void loadFinal()
{
    // Sol
    Point* s1 = new Point({0, 200, 0}, {0,0,0}, 0.0f);
    Point* s2 = new Point({GetScreenWidth(), 200, 0}, {0,0,0}, 0.0f);
    Point* s3 = new Point({GetScreenWidth(), 250, 0}, {0,0,0}, 0.0f);
    Point* s4 = new Point({0, 250, 0}, {0,0,0}, 0.0f);

    Edge* se1 = new Edge(*s1, *s2);
    Edge* se2 = new Edge(*s2, *s3);
    Edge* se3 = new Edge(*s3, *s4);
    Edge* se4 = new Edge(*s4, *s1);

    groundBody = new Body(
        {s1, s2, s3, s4},
        {se1, se2, se3, se4},
        {},
        false,
        true 
    );

    // Mur
    float groundY = 250; // Juste au dessus du sol
    float recWidth = 100.0f;
    float recHeight = 100.0f;
    float recStartX = 250.0f; // Positionné pour intercepter la trajectoire après le toboggan

    Point* c2_1 = new Point({recStartX,            groundY,             0}, {0,0,0}, 0.0f); // BG
    Point* c2_2 = new Point({recStartX + recWidth, groundY,             0}, {0,0,0}, 0.0f); // BD
    Point* c2_3 = new Point({recStartX + recWidth, groundY + recHeight, 0}, {0,0,0}, 0.0f); // HD
    Point* c2_4 = new Point({recStartX,            groundY + recHeight, 0}, {0,0,0}, 0.0f); // HG

    Edge* ec2_1 = new Edge(*c2_1, *c2_2);
    Edge* ec2_2 = new Edge(*c2_2, *c2_3);
    Edge* ec2_3 = new Edge(*c2_3, *c2_4);
    Edge* ec2_4 = new Edge(*c2_4, *c2_1);
    //Edge* ec2_d = new Edge(*c2_1, *c2_3); // Diagonale de rigidité

    finalWall = new Body({c2_1, c2_2, c2_3, c2_4}, {ec2_1, ec2_2, ec2_3, ec2_4}, {}, false, true);

    // Ragdoll
    float x = GetScreenWidth()/2, y = 500;

    //Point* head   = new Point({x, y + 30, 0}, {0,0,0}, 0.8f);  // Une tête pour l'équilibre
    Point* neck   = new Point({x, y, 0}, {0,0,0}, 1.0f);
    Point* l_shdr = new Point({x - 45, y - 10, 0}, {0,0,0}, 1.0f);
    Point* r_shdr = new Point({x + 45, y - 10, 0}, {0,0,0}, 1.0f);
    Point* l_hip  = new Point({x - 20, y - 100, 0}, {0,0,0}, 1.0f);
    Point* r_hip  = new Point({x + 20, y - 100, 0}, {0,0,0}, 1.0f);

    // Membres
    Point* l_elbow = new Point({x - 80, y - 30, 0}, {0,0,0}, 1.2f);
    Point* l_hand  = new Point({x - 110, y - 50, 0}, {0,0,0}, 1.2f);
    Point* r_elbow = new Point({x + 80, y - 30, 0}, {0,0,0}, 1.2f);
    Point* r_hand  = new Point({x + 110, y - 50, 0}, {0,0,0}, 1.2f);

    Point* l_knee = new Point({x - 25, y - 160, 0}, {0,0,0}, 1.2f);
    Point* l_foot = new Point({x - 25, y - 220, 0}, {0,0,0}, 1.2f);
    Point* r_knee = new Point({x + 25, y - 160, 0}, {0,0,0}, 1.2f);
    Point* r_foot = new Point({x + 25, y - 220, 0}, {0,0,0}, 1.2f);

    auto makeEdge = [](Point* a, Point* b, float stiff = 1.0f) {
        return new Edge(*a, *b, Vector3Distance(a->pos_(), b->pos_()), stiff);
    };

    std::vector<Point*> pts = {neck, l_shdr, r_shdr, l_hip, r_hip, l_elbow, l_hand, r_elbow, r_hand, l_knee, l_foot, r_knee, r_foot };

    std::vector<Edge*> edges;

    //edges.push_back(makeEdge(head, neck));
    edges.push_back(makeEdge(neck, l_shdr));
    edges.push_back(makeEdge(neck, r_shdr));
    edges.push_back(makeEdge(l_shdr, r_shdr));
    edges.push_back(makeEdge(l_shdr, l_hip, 0.05f));
    edges.push_back(makeEdge(r_shdr, r_hip, 0.05f));
    edges.push_back(makeEdge(l_hip, r_hip));
    
    // Diagonales de buste (Crucial pour la solidité)
    edges.push_back(makeEdge(l_shdr, r_hip, 0.05f));
    edges.push_back(makeEdge(r_shdr, l_hip, 0.05f));

    // --- 3. MEMBRES ARTICULÉS ---
    edges.push_back(makeEdge(l_shdr, l_elbow)); edges.push_back(makeEdge(l_elbow, l_hand));
    edges.push_back(makeEdge(r_shdr, r_elbow)); edges.push_back(makeEdge(r_elbow, r_hand));
    edges.push_back(makeEdge(l_hip, l_knee));   edges.push_back(makeEdge(l_knee, l_foot));
    edges.push_back(makeEdge(r_hip, r_knee));   edges.push_back(makeEdge(r_knee, r_foot));

    // --- 4. MUSCLES DE SOUTIEN (Stiffness basse) ---
    // Ces ressorts empêchent les bras et jambes de pendre comme des cordes mortes
    // Ils essaient de ramener les membres vers une pose en T naturelle
    edges.push_back(makeEdge(neck, l_elbow, 0.1f));
    edges.push_back(makeEdge(neck, r_elbow, 0.1f));
    edges.push_back(makeEdge(l_shdr, l_knee, 0.05f));
    edges.push_back(makeEdge(r_shdr, r_knee, 0.05f));

    ragdollBody = new Body(pts, edges, {}, true, false);

    world->bodies.push_back(groundBody);
    world->bodies.push_back(finalWall);
    world->bodies.push_back(ragdollBody);
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
    //Point* s1 = new Point({0, 0, 0}, {0,0,0}, 0.0f);
    //Point* s2 = new Point({800, 0, 0}, {0,0,0}, 0.0f);
    //Point* s3 = new Point({800, 50, 0}, {0,0,0}, 0.0f);
    //Point* s4 = new Point({0, 50, 0}, {0,0,0}, 0.0f);
//
    //Edge* se1 = new Edge(*s1, *s2);
    //Edge* se2 = new Edge(*s2, *s3);
    //Edge* se3 = new Edge(*s3, *s4);
    //Edge* se4 = new Edge(*s4, *s1);
//
    //Body* ground = new Body(
    //    {s1, s2, s3, s4},
    //    {se1, se2, se3, se4},
    //    {},
    //    false,  // ⚠️ IMPORTANT
    //    true    // freeze
    //);
    //groundBody = ground;
    world = new World({gravity, wind}, {}, 0.0f, 0.016f);
}

// ----------------------
// SCENE SWITCH
// ----------------------

void deleteAllBodies(){
    for (Body* b : Body::bodies) {
        delete b;
    }
    Body::bodies.clear();
}

// void loadScene(int scene)
// {
//     selectedPoint = nullptr;
//     //isDragging = false;
//     mouseState = IDLE;
//     SetMouseCursor(MOUSE_CURSOR_DEFAULT);
    
//     if (world != nullptr) {
//         delete world;
//         world = nullptr;
//     }
//     deleteAllBodies();

//     switch(scene)
//     {
//         case PENDULUM: loadPendulum(); break;
//         case RAGDOLL:  loadGround();  break;
//     }
// }


int main()
{

    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Physics Engine");
    SetTargetFPS(60);


    int numeroDiapo = 0;
    Vector3 diapoPos = Vector3{0.0f, 0.0f, 0.0f};
    
    Texture diapoText[NB_DIAPO];
    diapoText[0] = LoadTexture("./diapos/2.png");
    diapoText[1] = LoadTexture("./diapos/3.png");
    diapoText[2] = LoadTexture("./diapos/4.png");
    diapoText[3] = LoadTexture("./diapos/5.png");
    diapoText[4] = LoadTexture("./diapos/6.png");
    diapoText[5] = LoadTexture("./diapos/7.png");
    diapoText[6] = LoadTexture("./diapos/8.png");
    diapoText[7] = LoadTexture("./diapos/9.png");
    diapoText[8] = LoadTexture("./diapos/10.png");
    diapoText[9] = LoadTexture("./diapos/11.png");
    diapoText[10] = LoadTexture("./diapos/12.png");
    diapoText[11] = LoadTexture("./diapos/13.png");
    diapoText[12] = LoadTexture("./diapos/14.png");
    diapoText[13] = LoadTexture("./diapos/15.png");
    diapoText[14] = LoadTexture("./diapos/16.png");
    diapoText[15] = LoadTexture("./diapos/17.png");
    diapoText[16] = LoadTexture("./diapos/18.png");
    diapoText[17] = LoadTexture("./diapos/19.png");
    diapoText[18] = LoadTexture("./diapos/20.png");
    diapoText[19] = LoadTexture("./diapos/21.png");
    diapoText[20] = LoadTexture("./diapos/22.png");
    diapoText[21] = LoadTexture("./diapos/23.png");
    diapoText[22] = LoadTexture("./diapos/24.png");
    diapoText[23] = LoadTexture("./diapos/25.png");
    diapoText[24] = LoadTexture("./diapos/26.png");
    diapoText[25] = LoadTexture("./diapos/27.png");
    diapoText[26] = LoadTexture("./diapos/28.png");
    diapoText[27] = LoadTexture("./diapos/29.png");
    diapoText[28] = LoadTexture("./diapos/30.png");
    diapoText[29] = LoadTexture("./diapos/31.png");


    Rectangle sourceRec = { 0.0f, 0.0f, (float)diapoText[0].width, (float)diapoText[0].height };


    //loadGround();
    world = new World({gravity, wind}, {}, 0.0f, 0.016f);
    loadClothesline();
    
    boxEnableGround = true;
    running = true;
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        dt = fminf(dt, 0.016f);

        if (running) {
            // diapo
            Vector3 temp = diapoHanger->points[1]->pos_();
            diapoPos.x = temp.x;


            Vector2 mouse = GetMousePosition();

                // -------- INPUT --------
            if (IsKeyPressed(KEY_LEFT)) {
                Vector3 p; 
                p = diapoHanger->points[1]->pos_();
                p.x -= 2;
                diapoHanger->points[1]->pos_(p);
                p.x -= 2;
                diapoHanger->points[1]->oldPos_(p);

                if(numeroDiapo > 0) numeroDiapo--;
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                Vector3 p; 
                p = diapoHanger->points[1]->pos_();
                p.x += 2;
                diapoHanger->points[1]->pos_(p);
                p.x += 2;
                diapoHanger->points[1]->oldPos_(p);

                if(numeroDiapo < NB_DIAPO) numeroDiapo++;
            }

            if(numeroDiapo == 21)
            {
                if(!pendulumLoaded)
                {
                    loadPendulum();
                    pendulumLoaded = true;
                }
            }
            else
            {
                if(pendulumLoaded && pendulumPointer != nullptr)
                {
                    // 1. On le retire du vecteur du monde
                    auto& v = world->bodies;
                    v.erase(std::remove(v.begin(), v.end(), pendulumPointer), v.end());

                    // 2. On le retire du vecteur global des bodies (pour le dessin/clic)
                    auto& vg = Body::bodies;
                    vg.erase(std::remove(vg.begin(), vg.end(), pendulumPointer), vg.end());

                    // 3. On libère la mémoire
                    delete pendulumPointer;

                    pendulumPointer = nullptr;
                    pendulumLoaded = false;
                }
            }

            if(numeroDiapo == 22)
            {
                if(!springLoaded)
                {
                    loadSpring();
                    springLoaded = true;
                }
            }
            else
            {
                if(springLoaded && springPointer != nullptr)
                {
                    // 1. On le retire du vecteur du monde
                    auto& v = world->bodies;
                    v.erase(std::remove(v.begin(), v.end(), springPointer), v.end());

                    // 2. On le retire du vecteur global des bodies (pour le dessin/clic)
                    auto& vg = Body::bodies;
                    vg.erase(std::remove(vg.begin(), vg.end(), springPointer), vg.end());

                    // 3. On libère la mémoire
                    delete springPointer;

                    springPointer = nullptr;
                    springLoaded = false;
                }
            }
            if(numeroDiapo == 23)
            {
                if(!ballAndWallLoaded)
                {
                    loadBallAndWall();
                    ballAndWallLoaded = true;
                }
            }
            else
            {
                // Dans ton bloc de nettoyage (else numeroDiapo != 23)
                if(ballAndWallLoaded) {
                    auto& v = world->bodies;
                    auto& vg = Body::bodies;

                    // Supprimer Cube 1
                    if(cube1Pointer) {
                        v.erase(std::remove(v.begin(), v.end(), cube1Pointer), v.end());
                        vg.erase(std::remove(vg.begin(), vg.end(), cube1Pointer), vg.end());
                        delete cube1Pointer;
                        cube1Pointer = nullptr;
                    }
                    if(cube2Pointer) {
                        v.erase(std::remove(v.begin(), v.end(), cube2Pointer), v.end());
                        vg.erase(std::remove(vg.begin(), vg.end(), cube2Pointer), vg.end());
                        delete cube2Pointer;
                        cube2Pointer = nullptr;
                    }
                    if(ballPointer) {
                        v.erase(std::remove(v.begin(), v.end(), ballPointer), v.end());
                        vg.erase(std::remove(vg.begin(), vg.end(), ballPointer), vg.end());
                        delete ballPointer;
                        ballPointer = nullptr;
                    }
                    ballAndWallLoaded = false;
                }
            }

            if(numeroDiapo == 24)
            {
                if(!finalLoaded)
                {
                    loadFinal();
                    finalLoaded = true;
                }
            }
            else
            {
                // Dans ton bloc de nettoyage (else numeroDiapo != 23)
                if(finalLoaded) {
                    auto& v = world->bodies;
                    auto& vg = Body::bodies;

                    // Supprimer Cube 1
                    if(groundBody) {
                        v.erase(std::remove(v.begin(), v.end(), groundBody), v.end());
                        vg.erase(std::remove(vg.begin(), vg.end(), groundBody), vg.end());
                        delete groundBody;
                        groundBody = nullptr;
                    }
                    if(finalWall) {
                        v.erase(std::remove(v.begin(), v.end(), finalWall), v.end());
                        vg.erase(std::remove(vg.begin(), vg.end(), finalWall), vg.end());
                        delete finalWall;
                        finalWall = nullptr;
                    }
                    if(ragdollBody) {
                        v.erase(std::remove(v.begin(), v.end(), ragdollBody), v.end());
                        vg.erase(std::remove(vg.begin(), vg.end(), ragdollBody), vg.end());
                        delete ragdollBody;
                        ragdollBody = nullptr;
                    }
                    finalLoaded = false;
                }
            }

            // if (IsKeyPressed(KEY_LEFT)) {
            //     //isDragging = false;
            //     mouseState = IDLE;
            //     SetMouseCursor(MOUSE_CURSOR_DEFAULT);
            //     selectedPoint = nullptr;
            //     currentScene = (currentScene - 1 + SCENE_COUNT) % SCENE_COUNT;
            //     loadScene(currentScene);
            // }

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
                        SetMouseCursor(MOUSE_CURSOR_DEFAULT);
                        btnAddBodyPressed = false;

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
                        SetMouseCursor(MOUSE_CURSOR_DEFAULT);
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
                if(mouseState != TARGETING && mouseState != PLACING) {
                    mouseState = IDLE;
                    SetMouseCursor(MOUSE_CURSOR_DEFAULT);
                }
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

        if(boxEnableGravity) enableGravity();
        if(boxEnableWind) enableWind();
        if(boxEnableBullet) enableBullet();

        // -------- DRAW --------
    
        BeginDrawing();
        ClearBackground(RAYWHITE);

            Rectangle destRec = { diapoPos.x-GetScreenWidth()/2, diapoPos.y, GetScreenWidth(), GetScreenHeight() }; // Ici on force 100x100 pixels
            Vector2 origin = { 0.0f, 0.0f }; // Point de pivot (0,0 = en haut à gauche)
            DrawTexturePro(diapoText[numeroDiapo], sourceRec, destRec, origin, 0.0f, WHITE);

            for (Body* b : Body::bodies) b->Draw();
            
            if(selectedTarget) selectedTarget->Draw(GREEN);

            DrawText(TextFormat("Scene: %d", currentScene), 10, 10, 20, BLACK);

        EndDrawing();
    }

    for(int i = 0; i < NB_DIAPO; i++)
    {
        UnloadTexture(diapoText[i]);
    }
    
    CloseWindow();
    
    return 0;
}


/*
TO DO

Tester les limites du bouton run 
Mettre en forme
Delete Body

Add volume

Delete Volume

supprimer les bodies Out of bounds
*/