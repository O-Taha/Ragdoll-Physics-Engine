#include "raylib.h"

int main()
{
    InitWindow(800, 450, "Ragdoll Engine");

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        BeginDrawing();

        ClearBackground(RAYWHITE);
        DrawText("Raylib + Docker + Premake works!", 150, 200, 20, BLACK);

        EndDrawing();
    }

    CloseWindow();
}