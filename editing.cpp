#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
#include <limits>
using namespace parser;

typedef unsigned char RGB[3];

Vec3f operator+(const Vec3f& a, const Vec3f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3f operator-(const Vec3f& a, const Vec3f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3f operator*(const Vec3f& v, float s) {
    return {v.x * s, v.y * s, v.z * s};
}

float dot(const Vec3f& a, const Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3f normalize(const Vec3f& v) {
    float len = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len == 0) return {0, 0, 0};
    return {v.x / len, v.y / len, v.z / len};
}

Vec3f cross(const Vec3f& a, const Vec3f& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}
float determinant(float a,float b,float c,float d,float e,float f,float g,float h,float i){
    return a*(e*i-h*f) + b*(g*f-d*i) + c*(d*h-e*g);
}


struct HitRecord {
        

        Vec3f intersection_point;
        Material material;
        Vec3f normal;
        float t;
        bool is_intersected;
};

class Ray {
    Vec3f origin, direction;
    bool isShadow;
    public: 
        Ray(const Vec3f& orig, const Vec3f& dir) : origin(orig), direction(normalize(dir)), isShadow(false) {}
        Ray(const Vec3f& orig, const Vec3f& dir,const bool isShadow) : origin(orig), direction(normalize(dir)), isShadow(isShadow) {}
    //we can use this as: Ray ray = Ray::cameraToPixel(camera, pixel_x, pixel_y);
    //The cameraToPixel method is static because it creates a Ray without needing an instance of the Ray class. 
    //It uses the camera's parameters and pixel coordinates to calculate the origin and direction of the ray.
    Vec3f getDirection() const { return direction; }
    static Ray cameraToPixel(const Camera &camera, int pixel_x, int pixel_y) { //constların yeri hkk bak???
        float l = camera.near_plane.x ;
        float r = camera.near_plane.y;
        float b =camera.near_plane.z;
        float t = camera.near_plane.w;
        Vec3f v = normalize(camera.up);
        Vec3f w = normalize((camera.gaze)*(-1.0));
        Vec3f u = normalize(cross(v,w));
        Vec3f m = camera.position + (camera.gaze)*camera.near_distance ;
        float s_u = (pixel_x + 0.5)* (r-l) / camera.image_width;
        float s_v = (pixel_y + 0.5)* (t-b) / camera.image_height;
        Vec3f q = m + u*l + v*t ;
        Vec3f s = q + u*s_u - v*s_v ;
        return Ray(camera.position, normalize(s - camera.position));
    }


    HitRecord intersectionSphere(Sphere const &sphere, const Scene& scene) {
        HitRecord hit;
        hit.is_intersected = false;
        hit.t=std::numeric_limits<float>::infinity();
        Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];  //the first vertex_id is 1
        Vec3f o_c = this->origin-center;
        Vec3f d = this->direction;
        float A= dot(d,d);
        float B = dot(d,o_c)*2;
        float C = dot(o_c,o_c) - (sphere.radius)*(sphere.radius);
        float discriminant = (B*B - 4*A*C);
        if(discriminant>=0) {
            
            float t1 =(-B - sqrt(discriminant)) / (2*A);
            float t2 =(-B + sqrt(discriminant)) / (2*A);
            if(t1 < t2 && t1>0) hit.t = t1;
            else if (t2 < t1 && t2>0) hit.t = t2;
            else return hit;
            hit.material=scene.materials[sphere.material_id-1]; //burada key gibi veriyoruz sanırım o yüzden +1 yok
            hit.intersection_point = this->origin + d*hit.t ;
            hit.normal = normalize(hit.intersection_point - center);
            hit.is_intersected = true;
        }
        return hit;
    }

    HitRecord intersectionTriangle(Triangle const&triangle, const Scene& scene, const Camera&camera){
        HitRecord hit;
        hit.is_intersected = false;
        hit.t=std::numeric_limits<float>::infinity();
        Vec3f V1 = scene.vertex_data[triangle.indices.v1_id -1];
        Vec3f a_b = scene.vertex_data[triangle.indices.v0_id -1] - V1 ;
        hit.normal = cross(scene.vertex_data[triangle.indices.v2_id -1] - V1 ,a_b);
        if(isShadow==false && dot(hit.normal,camera.gaze)>=0) return hit;
        float dirX = this->direction.x;
        float dirY = this->direction.y;
        float dirZ = this->direction.z;

        Vec3f a_c = (a_b+V1) - scene.vertex_data[triangle.indices.v2_id -1] ;
        Vec3f a_o = (a_b+V1) - this->origin ;
        float det_A = determinant(a_b.x,a_b.y,a_b.z,a_c.x,a_c.y,a_c.z,dirX,dirY,dirZ);
        float beta = determinant(a_o.x,a_o.y,a_o.z,a_c.x,a_c.y,a_c.z,dirX,dirY,dirZ) / det_A ;
        float gama = determinant(a_b.x,a_b.y,a_b.z,a_o.x,a_o.y,a_o.z,dirX,dirY,dirZ) / det_A ;
        float t = determinant(a_b.x,a_b.y,a_b.z,a_c.x,a_c.y,a_c.z,a_o.x,a_o.y,a_o.z) / det_A ;
        //if(fabs(det_A) < 1e-6) return hit;
        if (t>0 && beta+gama <= 1 && 0 <= beta && 0 <= gama){ //tmin<=t<=tmax ???
            hit.is_intersected = true;
            hit.material = scene.materials[triangle.material_id-1];
            hit.t = t;
            hit.intersection_point = this->origin + (this->direction)*(hit.t) ;
            hit.normal = normalize(hit.normal);
        }
        return hit;
    }

    HitRecord intersectionMesh(Mesh const&mesh, const Scene& scene, const Camera&camera){
        HitRecord hit;
        HitRecord current_hit;
        hit.is_intersected = false;
        hit.t=std::numeric_limits<float>::infinity();
        Triangle mesh_triangle;

        for(int i = 0; i<mesh.faces.size(); i++){
            mesh_triangle = {mesh.material_id, mesh.faces[i]};
            current_hit = intersectionTriangle(mesh_triangle,scene,camera);
            if(current_hit.is_intersected==true) {
                if(current_hit.t < hit.t) {hit = current_hit; }
            } 
        }
        return hit;
    }

    HitRecord find_closest_hit(Scene& scene, Camera&camera){
        HitRecord hit;
        HitRecord current_hit;
        hit.is_intersected = false;
        hit.t=std::numeric_limits<float>::infinity();
        for(int i=0; i< scene.spheres.size() ; i++){
            current_hit =intersectionSphere(scene.spheres[i],scene);
            if(current_hit.is_intersected==true) {
                if(current_hit.t < hit.t) {hit = current_hit; }
            } 
        }

        for(int i=0; i< scene.triangles.size() ; i++){
            current_hit =intersectionTriangle(scene.triangles[i],scene,camera);
            if(current_hit.is_intersected==true) {
                if(current_hit.t < hit.t) {hit = current_hit; }
            } 
        }

       for(int i=0; i< scene.meshes.size() ; i++){
            current_hit =intersectionMesh(scene.meshes[i],scene,camera);
            if(current_hit.is_intersected==true) {
                if(current_hit.t < hit.t) {hit = current_hit; }
            } 
        }

        return hit;
    }

 
};

float calculateDistance(Vec3f a, Vec3f b){
    return sqrt(pow(a.x - b.x ,2)+pow(a.y - b.y,2)+pow(a.z - b.z,2));
}
float calculateDistance2(Vec3f a, Vec3f b){
    return pow(a.x - b.x ,2)+pow(a.y - b.y,2)+pow(a.z - b.z,2);
}

Vec3f applyShading(Ray ray, int depth, HitRecord hit, Scene& scene, Camera&camera);
Vec3f compute_color(Ray ray,int depth, Scene& scene,Camera&camera);

bool inShadow(Vec3f intersection_point, PointLight I, Scene& scene,Camera&camera,Vec3f normal){ //kendisiyle çakışması ve arasında olmasının garantisi
        Vec3f shadowRayDir = normalize(I.position - intersection_point);
        Ray shadowRay = Ray((intersection_point+ normal * (scene.shadow_ray_epsilon)),shadowRayDir,true); //offseti sildim
        float objToLightDistance, objToObjDistance;
        objToLightDistance = calculateDistance2(intersection_point,I.position);
        HitRecord hit;
        hit.is_intersected = false;

        for(int i=0; i<scene.spheres.size(); i++){

            hit=shadowRay.intersectionSphere(scene.spheres[i],scene);

            if(hit.is_intersected 
            && calculateDistance2(hit.intersection_point,I.position) < objToLightDistance
            &&calculateDistance2(hit.intersection_point,intersection_point) < objToLightDistance ) {
            return true;}
            
        }
        //calculateDistance2(hit.intersection_point,intersection_point)>scene.shadow_ray_epsilon

        for(int i=0; i< scene.triangles.size(); i++){

            hit=shadowRay.intersectionTriangle(scene.triangles[i],scene,camera);

            if(hit.is_intersected 
            &&calculateDistance2(hit.intersection_point,I.position) < objToLightDistance
            &&calculateDistance2(hit.intersection_point,intersection_point) < objToLightDistance ) {
            return true;}
            
        }

        for(int i=0; i<scene.meshes.size(); i++){

            hit =shadowRay.intersectionMesh(scene.meshes[i],scene,camera);

            if(hit.is_intersected 
            && calculateDistance2(hit.intersection_point,I.position) < objToLightDistance
            &&calculateDistance2(hit.intersection_point,intersection_point) < objToLightDistance ) {
            return true;}
        }

        return false;
}

Vec3f applyShading(Ray ray, int depth, HitRecord hit, Scene& scene, Camera& camera){
    Vec3f color= {0,0,0};
    //ambient contribution
    color.x += (scene.ambient_light).x * (hit.material.ambient).x;
    color.y += (scene.ambient_light).y * (hit.material.ambient).y;
    color.z += (scene.ambient_light).z * (hit.material.ambient).z;
    
    //Doing the above is mathematically correct, but if you then try to raytrace from that ray, you may hit the same object you are reflecting off.
    // One way to fight that problem is to push the ray positin a small amount away from the surface to make sure it misses it. 
    //You can do that for example like this: ReflectRayLocation = ReflectRayLocation + ReflectRayDirection * 0.01

    Vec3f viewDirection =  ray.getDirection() * (-1.0);
    if(hit.material.is_mirror){
        float cos_theta=dot(viewDirection,hit.normal);
        Vec3f reflection_ray_direction = viewDirection *(-1.0f) + hit.normal*(cos_theta*(2.0f));
        Ray reflection_ray = Ray(hit.intersection_point + reflection_ray_direction * scene.shadow_ray_epsilon , reflection_ray_direction,true);

        Vec3f computed_color=compute_color(reflection_ray , depth+1,scene,camera);
        //printf("%f %f %f \n",computed_color.x,computed_color.y,computed_color.z);
        color.x += computed_color.x * hit.material.mirror.x ;
        color.y += computed_color.y * hit.material.mirror.y ;
        color.z += computed_color.z * hit.material.mirror.z ;
    }

    for(int i=0; i<scene.point_lights.size(); i++){
        if(!inShadow(hit.intersection_point,scene.point_lights[i],scene,camera,hit.normal)){
                
                Vec3f objToLight = normalize(scene.point_lights[i].position-hit.intersection_point);
                float clampedDotProduct = fmax(0,dot(objToLight,normalize(hit.normal)));
                float d_square =pow(calculateDistance(scene.point_lights[i].position,hit.intersection_point),2);
                Vec3f lightIntensity = scene.point_lights[i].intensity;
                float diffuseRed = lightIntensity.x * clampedDotProduct * hit.material.diffuse.x /d_square;
                float diffuseGreen = lightIntensity.y * clampedDotProduct * hit.material.diffuse.y / d_square;
                float diffuseBlue = lightIntensity.z * clampedDotProduct * hit.material.diffuse.z / d_square;
                color.x+=diffuseRed;
                color.y+=diffuseGreen;
                color.z+=diffuseBlue;

                
                Vec3f viewDirection =  ray.getDirection() * (-1.0);
                Vec3f halfwayVector = normalize(objToLight + viewDirection);
                float clampedDotProduct2 = fmax(0, dot(halfwayVector, hit.normal)); //normalize et çok önemli
                float phongPart = pow(clampedDotProduct2, hit.material.phong_exponent);
                float specularRed = hit.material.specular.x * phongPart* lightIntensity.x  /d_square;
                float specularGreen = hit.material.specular.y * phongPart * lightIntensity.y  / d_square;
                float specularBlue = hit.material.specular.z * phongPart * lightIntensity.z / d_square;
                color.x+=specularRed;
                color.y+=specularGreen;
                color.z+=specularBlue;

        }
        

    }
    return color;
}


Vec3f compute_color(Ray ray,int depth, Scene& scene,Camera&camera){ //en son mainin içinde Vec3i'ye çevirip clample
        if(depth > scene.max_recursion_depth){
            return {0,0,0};
        }

        if(ray.find_closest_hit(scene,camera).is_intersected == true){
            return applyShading(ray,depth,ray.find_closest_hit(scene,camera),scene,camera);
        }

        else if(depth==0) { //Vec3i Vec3f'ye dönüştürüldü. Umarım loss olmamıştır.
            Vec3f background;
            background.x = scene.background_color.x;
            background.y = scene.background_color.y;
            background.z = scene.background_color.z;
            return {background};
        }

        else return {0,0,0};
}





int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    for(Camera& camera : scene.cameras){
    int width = camera.image_width;
    int height = camera.image_height;

    unsigned char* image = new unsigned char [width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {   
            Ray ray = Ray::cameraToPixel(camera,x,y);
            Vec3f color = compute_color(ray,0,scene,camera);
            
            //int roundedX = static_cast<int>(std::round(color.x));
            //int roundedY = static_cast<int>(std::round(color.y));
            //int roundedZ = static_cast<int>(std::round(color.z));

            image[i++] = color.x > 255 ? 255 : color.x;
            image[i++] = color.y > 255 ? 255 : color.y;
            image[i++] = color.z > 255 ? 255 : color.z;
        }
    }

    write_ppm(camera.image_name.c_str(), image, width, height);
    }
}