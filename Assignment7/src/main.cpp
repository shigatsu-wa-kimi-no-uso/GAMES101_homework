#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{

    // Change the definition here to change resolution
    Scene scene(784, 784);

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    //red->Ks = Vector3f(1) - red->Kd;
    //red->f0 = Vector3f(1, 0.71, 0.29);
    //red->roughness = 0.25;
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);
    /*
    Material* gold = new Material(MICROFACET, Vector3f(0.0f));
    gold->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    gold->Ks = Vector3f(1) - red->Kd;
    gold->f0 = Vector3f(1, 0.71, 0.29);
    gold->roughness = 0.25;*/

    Material* gold = new Material(MICROFACET, Vector3f(0.0f));
    gold->Ks = Vector3f(0.45, 0.45, 0.45);
    gold->Kd = Vector3f(0.3, 0.3, 0.25);
    gold->f0 = Vector3f(1, 0.71, 0.29);
    gold->roughness = 0.25;

    MeshTriangle floor("src/models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("src/models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("src/models/cornellbox/tallbox.obj", white);
    MeshTriangle left("src/models/cornellbox/left.obj", red);
    MeshTriangle right("src/models/cornellbox/right.obj", green);
    MeshTriangle light_("src/models/cornellbox/light.obj", light);


    Sphere sphere1(Vector3f(150, 100, 200), 100, gold);
    //scene.Add(&sphere1);
    
    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}