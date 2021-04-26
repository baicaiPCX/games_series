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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f wo=ray.direction;
    Intersection pos_light;
    float pdf;
    sampleLight(pos_light,pdf);
    Intersection m_inter=intersect(ray);
    if(not m_inter.happened){
        return backgroundColor;
    }

    // adjudge is blocked
    Vector3f ws=(pos_light.coords-m_inter.coords).normalized();
    Intersection is_block=intersect(Ray(m_inter.coords,ws));
    Vector3f Le;
    float cos_theta=dotProduct(m_inter.normal,ws);
    if(is_block.obj==pos_light.obj){
        Vector3f f_re=m_inter.m->eval(ws,wo,m_inter.coords);
        float cos_theta1=dotProduct(pos_light.normal,ws);
        Vector3f emit=pos_light.emit;
        Vector3f x=pos_light.coords-m_inter.coords;
        float xx=dotProduct(x,x);
        for(int i=0;i<3;++i){
            Le[i]=f_re[i]*emit[i]*cos_theta*cos_theta/xx/pdf;
        }
    }

    // non-emit
    Vector3f L;
    Vector3f wi=m_inter.m->sample(wo,m_inter.normal);
    pdf=m_inter.m->pdf(wi,wo,m_inter.normal);
    float P_RR=get_random_float();
    if(P_RR<RussianRoulette){
        return L+Le;
    }
    Vector3f f_r=m_inter.m->eval(wi,wo,m_inter.coords);
    Vector3f I=castRay(Ray(m_inter.coords,wi),depth+1);
    cos_theta=dotProduct(m_inter.normal,wi);
    for(int i=0;i<3;++i){
        L[i]=f_r[i]*I[i]*cos_theta/pdf/RussianRoulette;
    }
    return L+Le;
}