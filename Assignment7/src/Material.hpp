//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include <assert.h>
enum MaterialType { DIFFUSE, MICROFACET};

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks, f0;
    float roughness;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wo, const Vector3f &N){
    switch(m_type){
    default:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
    }
}

inline float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
    default:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wi, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
    }
}


inline Vector3f lambert(const float cosNI,const Vector3f& albedo) {
    if (cosNI > 0.0f) {
        Vector3f diffuse = albedo / M_PI;   //理想均匀散射  推导可知fr_diff=1/pi   考虑颜色(即RGB各通道的吸收情况)后为fr_diff=albedo/pi
        return diffuse;
    } else {
        return Vector3f(0);
    }
}

inline float normalDistribution(float roughness,float cosNH) {
    //Trowbridge-Reitz GGX
    float alpha2 = roughness * roughness;
    float cosNH2 = cosNH * cosNH;
    float sinNH2 = 1 - cosNH2;
    float denominator = alpha2 * cosNH2 + sinNH2;
    denominator *= M_PI * denominator;
    return alpha2 / denominator;
}


inline float schlickGGX(float cosNV, float roughness) {
    //Schlick GGX
    float alphap1 = (roughness + 1);
    float alphap12 = alphap1 * alphap1;
    float kDir = alphap12 / 8.0;
    float denominator = cosNV * (1 - kDir) + kDir;
    if (denominator < 0.001) {
        return 1;
    }
    return cosNV / denominator;
}


inline float geometry(float cosNR,float cosNI,float roughness) {
    //Shadowing-masking
    return schlickGGX(cosNI, roughness) * schlickGGX(cosNR, roughness);
}


inline Vector3f fresnelSchlick(const Vector3f& f0,float cosNI) {
    float t = 1 - cosNI;
    float t2 = t * t;
    float t5 = t2 * t2 * t;
    return f0 + (Vector3f(1) - f0) * t5;
}

//注意数值计算问题！！
inline Vector3f cookTorrance(const Vector3f& kd,const Vector3f& ks,const Vector3f& f0,float cosNI,float cosNR,float cosNH,float roughness) {
    //颜色:漫反射albedo 镜面反射f0
    //能量分配:漫反射kd 镜面发射ks
    cosNI = std::max(cosNI, 0.0f);
    cosNR = std::max(cosNR, 0.0f);
    cosNH = std::max(cosNH, 0.0f);
    Vector3f fd = lambert(cosNI, f0);
    float d = normalDistribution(roughness, cosNH);
    float g = geometry(cosNR, cosNI, roughness);
    float denominator = 4 * cosNR * cosNI;
    if (denominator == 0) {
        denominator = 0.000001; //防止除0出现inf -inf 和nan
    }
    Vector3f f = fresnelSchlick(f0, cosNI);
    Vector3f kd_balance = Vector3f(1.0f) - f; //可以用kd,ks=1-kd平衡能量,也可以使用f代替ks,kd=1-f来保持能量守恒
    Vector3f diff = kd * fd;
    Vector3f spec =  ks * (d * f * g) / denominator;
    Vector3f c = spec + diff;
    return c;
}

//计算 fr (BRDF)
inline Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch (m_type) {
    case DIFFUSE:
    {
        // calculate the contribution of diffuse   model
        float cosNI = dotProduct(N, wi); //wi为入射光线,应该与法线夹角小于90才能发生漫反射
        return lambert(cosNI, Kd);
        break;
    }
    case MICROFACET:
    {
        float cosNR = dotProduct(N, wo);
        float cosNI = dotProduct(N, wi);
        float cosNH = dotProduct(N, normalize(wi + wo));
        return cookTorrance(Kd, Ks, f0, cosNI, cosNR, cosNH, roughness);
        break;
    }
    }
}

#endif //RAYTRACING_MATERIAL_H
