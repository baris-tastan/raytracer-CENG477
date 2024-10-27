#include <iostream>
#include "parser.h"
#include "ppm.h"
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
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
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
class Ray {
    Vec3f origin, direction;

    Ray(const Vec3f& orig, const Vec3f& dir) : origin(orig), direction(normalize(dir)) {}

    //we can use this as: Ray ray = Ray::cameraToPixel(camera, pixel_x, pixel_y);
    //The cameraToPixel method is static because it creates a Ray without needing an instance of the Ray class. 
    //It uses the camera's parameters and pixel coordinates to calculate the origin and direction of the ray.

    static Ray cameraToPixel(const Camera &camera, int pixel_x, int pixel_y) {
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
    struct HitRecord {
        bool is_intersected;
        float t;
        Vec3f intersection_point;
        Material material;
        Vec3f normal;
    };

    HitRecord intersectionSphere(Sphere const &sphere, const Scene& scene) {
        HitRecord hit;
        hit.is_intersected = false;
        Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];  //the first vertex_id is 1
        Vec3f o_c = this->origin-center;
        Vec3f d = this->direction;
        float A= dot(d,d);
        float B = dot(d,o_c)*2;
        float C = dot(o_c,o_c) - (sphere.radius)*(sphere.radius);
        float discriminant = (B*B - 4*A*C);
        if(discriminant>=0) {
            hit.is_intersected = true;
            float t1 =(-B - sqrt(discriminant)) / 2*A;
            float t2 =(-B + sqrt(discriminant)) / 2*A;
            hit.t = t1 < t2 ? t1 : t2;
            hit.material=scene.materials[sphere.material_id]; //burada key gibi veriyoruz sanırım o yüzden +1 yok
            hit.intersection_point = this->origin + d*hit.t ;
            hit.normal = hit.intersection_point - center;
        }
        return hit;
    }

    HitRecord intersectionTriangle(Triangle const&triangle, const Scene& scene){
        HitRecord hit;
        hit.is_intersected = false;
        Vec3f a_b = scene.vertex_data[triangle.indices.v0_id -1] - scene.vertex_data[triangle.indices.v1_id -1] ;
        Vec3f a_c = scene.vertex_data[triangle.indices.v0_id -1] - scene.vertex_data[triangle.indices.v2_id -1] ;
        Vec3f a_o = scene.vertex_data[triangle.indices.v0_id] - this->origin ;
        float det_A = determinant(a_b.x,a_b.y,a_b.z,a_c.x,a_c.y,a_c.z,this->direction.x,this->direction.y,this->direction.z);
        float beta = determinant(a_o.x,a_o.y,a_o.z,a_c.x,a_c.y,a_c.z,this->direction.x,this->direction.y,this->direction.z) / det_A ;
        float gama = determinant(a_b.x,a_b.y,a_b.z,a_o.x,a_o.y,a_o.z,this->direction.x,this->direction.y,this->direction.z) / det_A ;
        float t = determinant(a_b.x,a_b.y,a_b.z,a_c.x,a_c.y,a_c.z,a_o.x,a_o.y,a_o.z) / det_A ;
        if (beta+gama <= 1 && 0 <= beta && 0 <= gama && t>0){ //tmin<=t<=tmax ???
            hit.is_intersected = true;
            hit.material = scene.materials[triangle.material_id];
            hit.t = t;
            hit.intersection_point = this->origin + (this->direction)*(hit.t) ;
            hit.normal = cross(scene.vertex_data[triangle.indices.v2_id -1] - scene.vertex_data[triangle.indices.v1_id -1] ,a_b);
        }
        return hit;
    }

    HitRecord intersectionMesh(Mesh const&mesh){

    }

    //compute_color(Ray r, int depth) içinde,, o rayi scenedeki tüm objelerle kesiştirmeye çalışacağız 
    //(all spheres loop, all triangles loop,...) hit record tutacağız bir yandan ve bu kesişimin (hitrecord.is_intersected==true) ise
    // ve bir önceki kesişimden daha yakınsa(Hitrecord.t < current_min_t) hit recordu güncelle.
    //tüm objeleri döndüğünde elinde kalan hitrecord en yakın hit olmuş olacak.
    //daha sonra bu ray'i ve hitrecordunu shading fonskiyonuna gönder. shading fonksiyonu recursive, depthi artırarar compute_color'u geri çağırabilir.


    //mainin içinde, cameradan tüm pixelleri dön, cameratopixel'i çağır, rayi elde et. Depthi 0 vererek compute_color'u her ray için çağır.
    //Compute color bir renk dönecek, onu da 0,255 arasına sıkıştır. SON.
};

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.
    //
    // Normally, you would be running your ray tracing
    // code here to produce the desired image.

    const RGB BAR_COLOR[8] =
    {
        { 255, 255, 255 },  // 100% White
        { 255, 255,   0 },  // Yellow
        {   0, 255, 255 },  // Cyan
        {   0, 255,   0 },  // Green
        { 255,   0, 255 },  // Magenta
        { 255,   0,   0 },  // Red
        {   0,   0, 255 },  // Blue
        {   0,   0,   0 },  // Black
    };

    int width = 640, height = 480;
    int columnWidth = width / 8;

    unsigned char* image = new unsigned char [width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int colIdx = x / columnWidth;
            image[i++] = BAR_COLOR[colIdx][0];
            image[i++] = BAR_COLOR[colIdx][1];
            image[i++] = BAR_COLOR[colIdx][2];
        }
    }

    write_ppm("test.ppm", image, width, height);

}
