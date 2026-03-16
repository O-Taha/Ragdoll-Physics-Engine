#include <iostream>
#include <vector>
#include "raylib.h"

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
        void vel_(const Vector2& v) { vel = v; }
        void acc_(const Vector2& v) { acc = v; }
        void w_(float v) { w = v; }

};



int main()
{
    InitWindow(800, 450, "Ragdoll Engine");

    SetTargetFPS(60);
    Point p1({0,0}, {0,0});
    float i = 0;
    while (!WindowShouldClose())
    {
        p1.pos_((Vector2){++i, i});
        std::cout << "pos: " << p1.pos_().x << std::endl;
        BeginDrawing();

        ClearBackground(RAYWHITE);
        DrawText("Point!", 150, 200, 20, BLACK);
        DrawCircleV(p1.pos_(), 10, RED);
        EndDrawing();
    }

    CloseWindow();
}