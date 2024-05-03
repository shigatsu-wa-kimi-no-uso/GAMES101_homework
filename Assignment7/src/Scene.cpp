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

//wo是出射方向,方向与hit所储存的击中点法向量夹角小于90
Vector3f Scene::shade(const Intersection& hit, const Vector3f& wo) const {
    Vector3f l_dir(0, 0, 0);
    //1)对光源采样,随机获得一条入射光线
    Intersection il;
    float il_pdf;
    sampleLight(il, il_pdf);
    Vector3f n = hit.normal.normalized();
    Vector3f p = hit.coords;
    Vector3f x = il.coords;
    Vector3f nn = il.normal.normalized();
    Vector3f emit = il.emit;
    //缓解精度问题导致的自遮挡问题
    Vector3f p_deviation = (dotProduct(wo, n) > 0) ?
        p + n * EPSILON * 100:
        p - n * EPSILON * 100;
    Vector3f incidentVec = p_deviation - x; //从光源指向反射点
    Vector3f wi = (-incidentVec).normalized();
    //2)遮挡检测
    Intersection testI = intersect(Ray(p_deviation, wi, 0)); //注意调整方向
    if (testI.happened == true && testI.m->hasEmission()) {
        //3)立体角->面积微元
        float dist = incidentVec.norm();
        float dist2 = dist * dist;
        incidentVec = incidentVec.normalized();
        float cos_theta1 = dotProduct(incidentVec, nn);
        float cos_theta = dotProduct(wi, n);

        //4)计算直接光照
        Vector3f fr = hit.m->eval(wi, wo, n);
      //  Vector3f fr = hit.m->eval2(-wo,wi, n);
        l_dir = emit * fr * cos_theta * cos_theta1 / dist2;
        l_dir = l_dir / il_pdf;
    }

    
    //5)计算间接光照
    Vector3f l_indir(0, 0, 0); 
    if (get_random_float() <= RussianRoulette) {
        Vector3f wi_indir = hit.m->sample(wo, n); //返回的是朝外(与法线夹角小于90)的向量
        Intersection nextI = intersect(Ray(p_deviation, wi_indir)); 
        if (nextI.happened && nextI.m->hasEmission() == false) {
            Vector3f li = shade(nextI,  -wi_indir);
            Vector3f fr = hit.m->eval(wi_indir, wo, n);
           // Vector3f fr = hit.m->eval2(-wo,wi_indir, n);
            //theta
            float theta = dotProduct(wi_indir, n);
            float obj_pdf = hit.m->pdf(wi_indir, wo, n);
            //float obj_pdf = hit.m->pdf2(-wo, wi_indir, n);
            //防止pdf过小导致放大光强
            if (obj_pdf > EPSILON) {
                l_indir = li * fr * theta / obj_pdf;
            }
        }
    }

    return hit.m->getEmission() + l_dir + l_indir / RussianRoulette; //不要漏了自发光项
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection hit = intersect(ray);
    if (hit.happened == false) {
        return backgroundColor;
    }
    //wo朝外,而不是朝向光线入射点
    Vector3f c = shade(hit, -ray.direction);
    clamp(0.0, 0.999, c);
    return c;
}
