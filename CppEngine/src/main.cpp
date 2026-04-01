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

    // Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);

        if (IsKeyPressed(KEY_Z)) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };

        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                DrawGrid(10, 1.0f);

            EndMode3D();

            DrawRectangle( 10, 10, 320, 93, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines( 10, 10, 320, 93, BLUE);

            DrawText("Free camera default controls:", 20, 20, 10, BLACK);
            DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, DARKGRAY);
            DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, DARKGRAY);
            DrawText("- Z to zoom to (0, 0, 0)", 40, 80, 10, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}