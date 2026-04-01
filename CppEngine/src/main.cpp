#include <iostream>
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

bool running = false;

class World;
class Body;
class Point;

class Point {   
    private:
        Vector2 pos;
        Vector2 oldPos;
        Vector2 vel;
        Vector2 acc;
        float w{1.0f}; // /!\ Inverse weight
        float oldW;
        std::vector<Vector2> recordPos;

    public:
        Point(Vector2 pos, Vector2 vel, float w = 1.0): pos(pos), vel(vel), w(w) {
            this->recordPos.push_back(pos);
        }

        const Vector2& pos_() const { return pos; }
        const Vector2& vel_() const { return vel; }
        const Vector2& acc_() const { return acc; }
        float w_() const { return w; }

        void pos_(const Vector2& v) { pos = v; }
        void incrementPos(float dx, float dy = 0) { 
            pos.x += dx; pos.y += dy;
        }
        void vel_(const Vector2& v) { vel = v; }
        void acc_(const Vector2& v) { acc = v; }
        void w_(float v) { w = 1/v; }
        void invW_(float v) { w = v; }

        void Draw(Color c = RED) {DrawCircleV(this->pos, 5, c);}
};

class Edge {
    public:
        Point& p1;
        Point& p2;
        float len;      // Resting length
        float stiff;    // Stiffness

        Edge(Point& p1, Point& p2, float l = -1, float s = 1): p1(p1), p2(p2), len(l), stiff(s) {
            if (l == -1) this -> len = Vector2Distance(p1.pos_(), p2.pos_()); // if no resting length is provided, compute it based on distance between points
        }
        void Draw(Color c = RED) {DrawLineV(this->p1.pos_(), this->p2.pos_(), c);}
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
        ) : points(points), edges(edges), forces(forces), wireframe(wireframe), freeze(freeze) {}

        void Draw() {
            for (auto& point : this->points) {
                point->Draw(BLUE);
            }
            for (auto& edge : this->edges) {
                edge->Draw(RED);
            }
        }
};
/* TODO ADD body.md !!!!!!!!!!!!!!!!!!!!!!!!!!!*/

void run(){}
void addBody(){}
void deleteBody(){}
void addVolume(){}
void deleteVolume(){}

void enableGround(){}
void enableGravity(){}
void enableWind(){}
void enableBullet(){}

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "ragdoll");

    SetTargetFPS(60);

    char textInput[96] = "test test test";
    bool textBoxEditMode = false;

    bool boxEnableGround = false;
    bool boxEnableGravity = false;
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

    bool btnRunPressed = false;

    while (!WindowShouldClose())
    {
        if(boxEnableGround) enableGround();
        if(boxEnableGravity) enableGravity();
        if(boxEnableWind) enableWind();
        if(boxEnableBullet) enableBullet();
        
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

            if(GuiButton((Rectangle){505, 60, 120, 30}, "#149#Add Body")) addBody();
            if(GuiButton((Rectangle){640, 60, 120, 30}, "#143#Del. Body")) deleteBody();

            if(GuiButton((Rectangle){505, 100, 120, 30}, "#162#Add Volume")) addVolume();
            if(GuiButton((Rectangle){640, 100, 120, 30}, "#143#Del. Body")) deleteVolume();
            
            if(GuiButton((Rectangle){505, 200, 120, 30}, "#142#Config Gravity")) btnConfigGravityWindow = !btnConfigGravityWindow;

            if(btnConfigGravityWindow)
            {
                DrawRectangle(150, 100, 200, 110, Fade(LIGHTGRAY, 0.3f));
            }

            if(GuiButton((Rectangle){505, 240, 120, 30}, "#142#Config Wind")) btnConfigWindWindow = !btnConfigWindWindow;

            if(btnConfigWindWindow)
            {
                DrawRectangle(150, 100, 200, 110, Fade(LIGHTGRAY, 0.3f));
            }

            if(GuiButton((Rectangle){505, 340, 120, 30}, "#142#Config Bullet")) btnConfigBulletWindow = !btnConfigBulletWindow;

            if(btnConfigBulletWindow)
            {
                DrawRectangle(150, 100, 200, 110, Fade(LIGHTGRAY, 0.3f));
            }
            
            if(GuiButton((Rectangle){505, 400, 120, 30}, "#134#Run")) run();
        EndDrawing();
    }

    CloseWindow();
    
    return 0;
}