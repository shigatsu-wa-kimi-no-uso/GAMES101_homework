//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

//wo是出射光线,方向与hit所储存的击中点法向量夹角小于90
Vector3f Scene::shade(const Intersection& hit, const Ray& wo) const {
    Vector3f l_dir(0, 0, 0);
    //1)对光源采样,随机获得一条入射光线
    Intersection il;
    float il_pdf;
    sampleLight(il, il_pdf);
    Vector3f incidentVec = hit.coords - il.coords; //从反射点指向光源
    //2)遮挡检测
    Intersection testI = intersect(Ray(hit.coords, -incidentVec, 0)); //注意调整方向
    if (testI.happened == false || testI.m->hasEmission()) {
        //3)立体角->面积微元
        float dist2 = pow(incidentVec.norm(), 2);
        incidentVec = incidentVec.normalized();
        float cos_theta1 = dotProduct(incidentVec, il.normal.normalized());
        float cos_theta = dotProduct(-incidentVec, hit.normal.normalized());

        //4)计算直接光照
        Vector3f fr = hit.m->eval(incidentVec, wo.direction, hit.normal);
        l_dir = il.emit * fr * cos_theta * cos_theta1 / dist2;
        l_dir = l_dir / il_pdf;
    }

    
    //5)计算间接光照
    Vector3f l_indir(0, 0, 0); 
    if (get_random_float() <= RussianRoulette) {
        Vector3f indir_incident = hit.m->sample(indir_incident, hit.normal); //返回的是朝外(与法线夹角小于90)的向量
        Intersection nextI = intersect(Ray(hit.coords, indir_incident)); 
        if (nextI.happened && nextI.m->hasEmission() == false) {
            Vector3f li = shade(nextI, Ray(hit.coords, -indir_incident));
            Vector3f fr = hit.m->eval(indir_incident, wo.direction, hit.normal);
            //theta
            float theta = dotProduct(indir_incident, hit.normal);
            float obj_pdf = hit.m->pdf(indir_incident, wo.direction, hit.normal);
            l_indir = li * fr * theta / obj_pdf;
        }
    }

    return l_dir + l_indir / RussianRoulette;
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection hit = intersect(ray);
    if (hit.happened == false) {
        return backgroundColor;
    }

    return shade(hit, Ray(ray.origin,-ray.direction));
}
