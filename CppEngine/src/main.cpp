#include <iostream>
#include <functional>
#include <vector>
#include "raylib.h"
#include "raymath.h"

#define CORRECTION_PASS 10

bool running = false;

class World;
class Body;
class Point;
class Edge;

enum SceneType { PENDULUM = 0, RAGDOLL, SCENE_COUNT };

int currentScene = 0;
World* world = nullptr;
std::vector<Body*> bodies;

Point* selectedPoint = nullptr;
bool isDragging = false;
float grabRadius = 15.0f;

inline Vector2 toVector2(const Vector3& v) { return {v.x, v.y}; }
inline Vector2 worldToScreen(const Vector3& v) { return {v.x, (float)GetScreenHeight() - v.y}; } // Used to invert y axis
Vector3 screenToWorld(Vector2 mouse) { return Vector3{ mouse.x, (float)GetScreenHeight() - mouse.y, 0 }; }

class Point {
    Vector3 pos, oldPos, vel, acc; float w{1.0f}; Body* owner;
public:
    Point(Vector3 pos, Vector3 vel, float w=1.0f) : pos(pos), vel(vel), w(w) {}
    const Vector3& pos_() const { return pos; }
    const Vector3& oldPos_() const { return oldPos; }
    const Vector3& vel_() const { return vel; }
    const Vector3& acc_() const { return acc; }
    float w_() const { return w; }
    void pos_(const Vector3& v) { pos=v; } void oldPos_(const Vector3& v) { oldPos=v; }
    void vel_(const Vector3& v) { vel=v; } void acc_(const Vector3& v) { acc=v; }
    void invW_(float v) { w = v; } void owner_(Body* b) { owner=b; }
    const Body* owner_() const { return owner; }
    void Draw(Color c=RED) { DrawCircleV(worldToScreen(pos),5,c); }
};

class Edge {
public:
    Point &p1, &p2; float len, stiff;
    Edge(Point &a, Point &b, float l=-1,float s=1): p1(a),p2(b),len(l),stiff(s){ if(l==-1) len=Vector2Distance(toVector2(p1.pos_()),toVector2(p2.pos_())); }
    void Draw(Color c=RED){ DrawLineV(worldToScreen(p1.pos_()),worldToScreen(p2.pos_()),c); }
};

class Body {
public:
    std::vector<Point*> points; std::vector<Edge*> edges;
    std::vector<std::function<Vector3(World&, Body&, Point&)>> forces;
    bool wireframe, freeze;
    Body(std::vector<Point*> pts, std::vector<Edge*> edgs, std::vector<std::function<Vector3(World&, Body&, Point&)>> fcs, bool wf, bool fr) 
        : points(pts), edges(edgs), forces(fcs), wireframe(wf), freeze(fr) { for(auto p:points) p->owner_(this); }
    void Draw(){ for(auto& p:points) p->Draw(BLUE); for(auto& e:edges) e->Draw(RED); }
};

class World {
    std::vector<Body*> bodies; float t,h,T; std::vector<float> recordTime;
    void applyVerlet(Point& p){
        Body* b = const_cast<Body*>(p.owner_());
        Vector3 a0 = computeAccel(*b,p); p.acc_(a0);
        Vector3 np = Vector3Add(Vector3Add(p.pos_(), Vector3Scale(p.vel_(),h)), Vector3Scale(p.acc_(),(h*h)/2)); p.pos_(np);
        p.acc_(computeAccel(*b,p));
        Vector3 nv = Vector3Add(p.vel_(), Vector3Scale(Vector3Add(a0,p.acc_()),h/2)); p.vel_(nv);
    }
    Vector3 computeAccel(Body& b, Point& p){
        Vector3 tot={0,0,0};
        for(auto& f:globalForces) tot=Vector3Add(tot,f(*this,b,p));
        for(auto& f:b.forces) tot=Vector3Add(tot,f(*this,b,p));
        return (p.w_()!=0)? Vector3Scale(tot,p.w_()): Vector3{0,0,0};
    }
    void applyConstraints(Edge& e){
        Point &p1=e.p1,&p2=e.p2; float w1=p1.w_(),w2=p2.w_(); if(w1+w2==0) return;
        Vector3 d=Vector3Subtract(p2.pos_(),p1.pos_()); float len=Vector3Length(d); if(len<0.0001f) return;
        float err=(len-e.len)/len; Vector3 corr=Vector3Scale(d,err*e.stiff);
        p1.pos_(Vector3Add(p1.pos_(),Vector3Scale(corr,w1/(w1+w2))));
        p2.pos_(Vector3Subtract(p2.pos_(),Vector3Scale(corr,w2/(w1+w2))));
    }
public:
    std::vector<std::function<Vector3(World&, Body&, Point&)>> globalForces;
    World(std::vector<std::function<Vector3(World&, Body&, Point&)>> f,std::vector<Body*> b,float T_=0,float h_=1.0f): globalForces(f),bodies(b),t(0),T(T_),h(h_){}
    void runStep(float dt=-1){
        h=(dt<=0)?h:dt;
        for(Body*& b:bodies){
            if(!b->wireframe) continue;
            if(!b->freeze) for(Point*& p:b->points){ p->oldPos_(Vector3(p->pos_())); applyVerlet(*p); }
            for(int i=0;i<CORRECTION_PASS;i++){
                for(Point*& p:b->points) for(Body*& pb:bodies) if(pb!=b && !pb->wireframe) checkCollisionBetween(*pb,*p);
                for(Edge*& e:b->edges) applyConstraints(*e);
            }
            for(Point*& p:b->points) p->vel_((p->pos_()-p->oldPos_())/h);
        }
        t+=h; recordTime.push_back(t);
    }

    void checkCollisionBetween(Body& vol, Point& p){
        if(vol.points.size()<4) return;
        Vector3 A=vol.points[0]->pos_(),B=vol.points[1]->pos_(),D=vol.points[3]->pos_();
        Vector3 u=Vector3Subtract(B,A),v=Vector3Subtract(D,A),ap=Vector3Subtract(p.pos_(),A);
        float ulen=Vector3DotProduct(u,u),vlen=Vector3DotProduct(v,v); if(ulen==0||vlen==0) return;
        float mu=Vector3DotProduct(ap,u)/ulen,mv=Vector3DotProduct(ap,v)/vlen;
        if(mu<0||mu>1||mv<0||mv>1) return;
        float dist_left=mu*Vector3Length(u),dist_right=(1-mu)*Vector3Length(u),dist_bottom=mv*Vector3Length(v),dist_top=(1-mv)*Vector3Length(v);
        float min_d=std::min({dist_left,dist_right,dist_bottom,dist_top});
        Vector3 u_norm=Vector3Normalize(u),v_norm=Vector3Normalize(v);
        if(min_d==dist_left) p.pos_(Vector3Subtract(p.pos_(),Vector3Scale(u_norm,dist_left)));
        else if(min_d==dist_right) p.pos_(Vector3Add(p.pos_(),Vector3Scale(u_norm,dist_right)));
        else if(min_d==dist_bottom) p.pos_(Vector3Subtract(p.pos_(),Vector3Scale(v_norm,dist_bottom)));
        else p.pos_(Vector3Add(p.pos_(),Vector3Scale(v_norm,dist_top)));
        Vector3 vel=Vector3Subtract(p.pos_(),p.oldPos_()); p.oldPos_(Vector3Add(p.oldPos_(),Vector3Scale(vel,0.9f)));
    }
};

auto gravity = [](World& world, Body& body, Point& point)->Vector3{ return (point.w_()!=0)? Vector3{0.0f,-981.0f/point.w_(),0.0f}:Vector3{0,0,0}; };
Point* getPointUnderCursor(Vector2 m){ for(auto b:bodies) for(auto p:b->points) if(Vector3Distance(p->pos_(),screenToWorld(m))<grabRadius) return p; return nullptr; }

void freeBodies(){
    if(selectedPoint) selectedPoint=nullptr;
    if(world){ delete world; world=nullptr; }
    for(auto b:bodies){
        for(auto p:b->points) delete p;
        for(auto e:b->edges) delete e;
        delete b;
    }
    bodies.clear();
}

void loadPendulum(){
    freeBodies();
    Point* p1=new Point({400,400,0},{0,0,0},0.0f);
    Point* p2=new Point({350,300,0},{0,0,0},1.0f);
    Edge* e1=new Edge(*p1,*p2,100.0f,1.0f);
    Body* body=new Body({p1,p2},{e1},{},true,false);
    body->forces.push_back(gravity);
    bodies.push_back(body);
    world=new World({gravity},bodies,0.0f,0.016f);
}

void loadRagdoll(){
    freeBodies();
    float x=400,y=350;
    Point *head=new Point({x,y,0},{0,0,0},1.0f),*torso=new Point({x,y-60,0},{0,0,0},1.0f),*pelvis=new Point({x,y-120,0},{0,0,0},1.0f);
    Point *l_arm=new Point({x-80,y-60,0},{0,0,0},1.0f),*r_arm=new Point({x+80,y-60,0},{0,0,0},1.0f);
    Point *l_leg=new Point({x-20,y-200,0},{0,0,0},1.0f),*r_leg=new Point({x+20,y-200,0},{0,0,0},1.0f);
    auto makeEdge=[](Point* a, Point* b){ return new Edge(*a,*b,Vector3Distance(a->pos_(),b->pos_()),1.0f); };
    std::vector<Point*> pts={head,torso,pelvis,l_arm,r_arm,l_leg,r_leg};
    std::vector<Edge*> edges={ makeEdge(head,torso),makeEdge(torso,pelvis),makeEdge(torso,l_arm),makeEdge(torso,r_arm),makeEdge(pelvis,l_leg),makeEdge(pelvis,r_leg) };
    Body* body=new Body(pts,edges,{},true,false); body->forces.push_back(gravity); bodies.push_back(body);
    Point *s1=new Point({0,0,0},{0,0,0},0.0f),*s2=new Point({800,0,0},{0,0,0},0.0f),*s3=new Point({800,50,0},{0,0,0},0.0f),*s4=new Point({0,50,0},{0,0,0},0.0f);
    Edge *se1=new Edge(*s1,*s2),*se2=new Edge(*s2,*s3),*se3=new Edge(*s3,*s4),*se4=new Edge(*s4,*s1);
    Body* ground=new Body({s1,s2,s3,s4},{se1,se2,se3,se4},{},false,true); bodies.push_back(ground);
    world=new World({gravity},bodies,0.0f,0.016f);
}

void loadScene(int scene){
    freeBodies();
    switch(scene){ case PENDULUM: loadPendulum(); break; case RAGDOLL: loadRagdoll(); break; }
}

int main(){
    InitWindow(800,450,"Physics Engine"); SetTargetFPS(60);
    loadScene(currentScene); running=true;
    while(!WindowShouldClose()){
        float dt=GetFrameTime(); if(dt>0.05f) dt=0.016f;
        if(IsKeyPressed(KEY_SPACE)) running=!running;
        if(IsKeyPressed(KEY_RIGHT)){ currentScene=(currentScene+1)%SCENE_COUNT; loadScene(currentScene); }
        if(IsKeyPressed(KEY_LEFT)){ currentScene=(currentScene-1+SCENE_COUNT)%SCENE_COUNT; loadScene(currentScene); }
        if(running){
            Vector2 mouse=GetMousePosition();
            if(IsMouseButtonPressed(MOUSE_LEFT_BUTTON)){
                selectedPoint=getPointUnderCursor(mouse);
                if(selectedPoint){ isDragging=true; selectedPoint->invW_(0.0f); }
            }
            if(IsMouseButtonReleased(MOUSE_LEFT_BUTTON)){
                if(selectedPoint){ selectedPoint->invW_(1.0f); selectedPoint=nullptr; }
                isDragging=false;
            }
            if(isDragging && selectedPoint){
                Vector3 m=screenToWorld(mouse);
                selectedPoint->pos_(m); selectedPoint->oldPos_(m);
            }
            world->runStep(dt);
        }
        BeginDrawing(); ClearBackground(RAYWHITE);
        for(auto b:bodies) b->Draw();
        DrawText(TextFormat("Scene: %d",currentScene),10,10,20,BLACK);
        EndDrawing();
    }
    freeBodies();
    CloseWindow();
}