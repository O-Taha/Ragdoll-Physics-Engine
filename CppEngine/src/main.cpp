#include <iostream>
#include <stdio.h>
#include <vector>
#include "raylib.h"
#include "raymath.h"

/*===========[ How to run ]===========
cd CppEngine/              ;
xhost +local:docker        ;             
docker compose run --rm dev

[Wait for container to launch...]

premake5 gmake2            ;             
make                       ;             
./bin/Debug/RagdollEngine               
/====================================*/

bool running = false;

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
            if (l == -1) this -> len = Vector2Distance(p1.pos_(), p2.pos_());
        }
        void Draw(Color c = RED) {DrawLineV(this->p1.pos_(), this->p2.pos_(), c);}
};

int main()
{
    InitWindow(800, 450, "Ragdoll Engine");

    SetTargetFPS(60);

    Point p1({0,20}, {0,0});
    Point p2({800,400}, {0,0});
    Edge e(p1, p2);

    while (!WindowShouldClose())
    {
        p1.incrementPos(1);
        p2.incrementPos(-1);

        std::cout << "pos: " << p1.pos_().x << std::endl;
        std::cout << "pos2: " << p2.pos_().x << std::endl;
        BeginDrawing();

        ClearBackground(RAYWHITE);
        DrawText("Edge", 150, 200, 20, BLACK);
        p1.Draw();
        p2.Draw(BLUE);
        e.Draw();
        EndDrawing();
    }

    CloseWindow();
}