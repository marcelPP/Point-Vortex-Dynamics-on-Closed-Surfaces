#include "math.h"


// get the time factor. We call this a lot so we should write it here.
// this method is very taylored to the setup of the 2d vortex flow file.
float getdt(){
	float fps = attrib(0,"detail","fps",0);
	float ts = attrib(0,"detail","timeScale",0);
	return ts/fps;
}	


// pick rainbow col in interval
vector rainbowColor(
    float min; // rainbow start
    float max; // rainbow end val
    float periods; // periods in rainbow
    float x){ // value to sample with 

    float pi2=3.14159265359*2;
    float f = pi2*periods/2;
    
    // Color code mapping
    //float c = f@speed/f@maxSpeed;
    float t = 2.0/(max-min)*x - (max+min)*1.0/(max-min);
    
    // compute colors
    float r = sin(t*f);
    float g = sin(t*f + pi2*1/3);
    float b = sin(t*f + pi2*2 /3 );
    // shift sine waves
    r=r*0.5+0.5;g=g*0.5+0.5;b=b*0.5+0.5;
    return set(r,g,b);
}

// oscillates a value r between [0,1] I times, with phase shift
float oscillate(float r,I,shift){
    return 0.5 + .5*sin( r*(6.28318530718)*I + shift);
}

float HeronFormula(float a; float b ; float c){
    // computes the area of a triangle given it's edge lengths
    float p= 0.5*(a+b+c);
    return sqrt(p*(p-a)*(p-b)*(p-c));
}


float getTriangleArea(int primitiveID){
    int geo = 0; // default geometry
    // grab any one edge around the triangle
    int halfEdge = primhedge(geo,primitiveID);
    // read all 3 vertices around the triangle
    int v1 = hedge_srcvertex(geo,halfEdge);
    int v2 = hedge_dstvertex(geo,halfEdge);
    int v3 = hedge_presrcvertex(geo,halfEdge);
    // extrac all edge lengths from these vertices
    float a = attrib(geo,'vertex','edgelength',v1);
    float b = attrib(geo,'vertex','edgelength',v2);
    float c = attrib(geo,'vertex','edgelength',v3);
    // apply Heros formula
    return HeronFormula(a,b,c);    
}

// averages the attribute of neighbour primitives to the point
// vector valued
void averagePrimToPoint(
        export vector attrib; // to store result
        int ptId; // id of point 
        string attributeName // attribute to call for
        ){
    int neighbourPrims[] = pointprims(0, ptId);
    vector vec = set(0,0,0);
    foreach (int i ; neighbourPrims){
        vec += attrib(0,"primitive",attributeName,i);
    }
    attrib = vec/len(neighbourPrims);
}

// averages the attribute of neighbour primitives to the point
// vector valued, weights adjusted
void areaAveragePrimToPoint(
        export vector attrib; // to store result
        int ptId; // id of point 
        string attributeName // attribute to call for
        ){
    int neighbourPrims[] = pointprims(0, ptId);
    vector vec = set(0,0,0);
    float mass = 0;
    foreach (int i ; neighbourPrims){
        float area = attrib(0,'primitive','area',i);
        mass+= area;
        vector gr = attrib(0,"primitive",attributeName,i);
        vec += gr*area;
    }
    attrib = vec/mass;
}


// return the constant gradient on the faces
// needs triangle geometry
void gradientField(
        export vector grad; // grad attrib holder
        string attribName; // attrib Name to compute gradient from
        int geo; // geo id with the attribute
        int primNum // id of face
        ){

    // collect corners
    int he = primhedge(geo,primNum);
    vector P1 = attrib(geo,'point','P',hedge_dstpoint(geo,he));
    int p1p=hedge_dstpoint(geo,he);
    he = hedge_next(geo,he);
    vector P2= attrib(geo,'point','P',hedge_dstpoint(geo,he));
    int p2p=hedge_dstpoint(geo,he);
    he = hedge_next(geo,he);
    vector P3= attrib(geo,'point','P',hedge_dstpoint(geo,he));
    int p3p=hedge_dstpoint(geo,he);
    
    // get edges and area
    vector v1=P3-P2;
    vector v2=P1-P3;
    vector v3=P2-P1;
    vector norm=cross(v1,v2)/length(cross(v1,v2));
    float area=length(cross(v1,v2));
    
    // compute gradient
    grad=1/(2*area)*(
             attrib(geo,'point',attribName,p1p)*v1
            +attrib(geo,'point',attribName,p2p)*v2
            +attrib(geo,'point',attribName,p3p)*v3);
    matrix3 m=ident();
    vector axis=norm;
    float angle=PI/2;
    rotate(m,angle,axis);
    grad*=m;
}


//in: geometry int, 3 point ids
//out: a new triangle in the geometry
int addtriangle(const int g,p,q,r) {
  int f = addprim(g,"poly");
  addvertex(g,f,p);
  addvertex(g,f,q);
  addvertex(g,f,r);
  return f;
}


//in: geometry int, 2 point ids
//out: a new line in the geometry
int addline(const int geo, p1, p2){
    int prim = addprim(geo, "polyline");
    addvertex(geo,prim,p1);
    addvertex(geo,prim,p2);
    return prim;
}
