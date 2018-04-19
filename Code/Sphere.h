#include "math.h"
// Use most of this code on triangulated surfaces
// y axis is up!


float lengthSphere(vector a, b) {
	a=a/(length(a)+0.001);
	b=b/(length(b)+0.001);
	return acos(a.x*b.x + a.y*b.y + a.z*b.z);
}


float absa(const vector2 z) {
	return sqrt(z.x*z.x + z.y*z.y);
}

// as in the spherical coordinates but with y axis up!
float theta(vector p){
	return atan2(p.z ,p.x);
}

// as in the spherical coordinates but with y axis up!
float phi(vector p){
	return acos(p.y);
}

void test(const float a,b,c,d; export float s3){
	s3 = a+b+c+d;
}

float abs(const vector2 z) {
	return sqrt(z.x*z.x + z.y*z.y);
}

// use spherical coordinates to color the sphere
vector sphericalRainbow(vector Pos){
    // normalize for safety
    Pos=Pos/length(Pos);
    float pi2=3.14159265359*2;
    // sphere coords
    float t=theta(Pos);
    float h=phi(Pos);
    float len = pi2 ; //angle;
    float f = pi2/len;
    // compute colors
    float r = sin(t*f+h*f*2);
    float g = sin(t*f-h*f*2 + pi2*1/3);
    float b = sin(t*f+h*f*2 + pi2*2 /3 );
    // shift sine waves
    r=r*0.5+0.5;g=g*0.5+0.5;b=b*0.5+0.5;
    return set(r,g,b);
}

// orient an (0,1,0) pointing object to the
//new normal and rotate nicley
vector4 orientForCopy(
    vector normal; // final pointing direction
    float rotation // rotation on that final direction
    ){
    // default
    vector objectUp = set(0,1,0);

    // compute quaternions for rotation and orientation seperatly
    vector4 Qrotate = quaternion(rotation,objectUp);
    vector4 Qorient = dihedral(objectUp,normal);

    // collect
    return qmultiply(Qorient,Qrotate); // combine rotations

}



//0000000000000000000000000000000000000000000000 INTERPOLATION RELATED CODE
//0000000000000000000000000000000000000000000000 INTERPOLATION RELATED CODE
//0000000000000000000000000000000000000000000000 INTERPOLATION RELATED CODE





// function to compute parameterization coordinates.
// returns set(s, t): p = a1+s(a2-a1)+t(a3-a1)
// but first it projects p to the triangle plane
vector2 planeParameterizationCoordinates(vector a1, a2, a3, p){
    // do barcenter interpol
    vector A2 = a2 - a1;
    vector A3 = a3 - a1;
    // now project to the plane of the triangle
    vector n = cross(A3,A2); n=n/length(n);
    p = p - n*dot(n,p-a1 );
    vector F = p - a1;
    
    // now compute t and s
    float denom = dot(F,A3)*dot(A2,A2)-dot(F,A2)*dot(A3,A2);
    float nom = dot(A3,A3)*dot(A2,A2)-dot(A3,A2)*dot(A3,A2);
    float t = denom/nom;
    float s = (dot(F,A2)-t*dot(A3,A2)) / (dot(A2,A2));    
    return set(s,t);
}




//input geometry of sphere, spherical coordinates
//output: s,t barycenter coords and prim id to get there
void getSTandPrimOfBarycenterInterpolation(int geoSp; float theta,phi; export vector2 chosenCoords; export int chosenPrim){
	// grab one of the nearest points:
    vector position = set ( cos(theta)*sin(phi),
                            cos(phi), // !!!!! z-y swap!
                            sin(theta)*sin(phi));
    float maxEdgeLength = attrib(geoSp,"detail","maxEdgeLength",0) + .00000001;
    int idNearPoint = nearpoint(geoSp, position, maxEdgeLength);
   
    // check  s,t coordinates of every prim neigbour
    int neighbourPrims[] = pointprims(geoSp, idNearPoint);
    chosenCoords = set(0,0);
    chosenPrim = -1; 
    float tCoords[] = neighbourPrims;
    float sCoords[] = neighbourPrims;
    int inTriangle = 0;
    int count=0;
    foreach (int primid; neighbourPrims) {
        if(inTriangle > .5) break; // stop if successfull
        // get corners of triangle
        int corners[] = primpoints(geoSp, primid);
        vector p1 = attrib(geoSp,"point","P",corners[0]);
        vector p2 = attrib(geoSp,"point","P",corners[1]);
        vector p3 = attrib(geoSp,"point","P",corners[2]);
        
        // get coords
        vector2 coords = planeParameterizationCoordinates(p1,p2,p3,position);
        float s=coords[0];
        float t=coords[1];        
    
        // boolean to quick check if inisde triangle
        //float ep = pow(10,-10000); // important on top of edges.
        int isIn = ( t>= 0 && t <= 1 && s>=0 && s <= 1 && t+s <= 1);
    
        // store prim
        if(isIn>0.5){
            // if found, keep it
            chosenCoords = coords;
            chosenPrim = primid;
            inTriangle = 1;
        }else{
            // else, collect to then analyse
            sCoords[count]=coords[0];
            tCoords[count]=coords[1];
            count+=1;
        }
     
    }
    
    // special case, if point ambigous on face, project to face.
    if( inTriangle < 0.5){
        float minImp=99999999999999;
        for (int id = 0 ; id < len(neighbourPrims); id++) {
            float s = sCoords[id];
            float t = tCoords[id];            
            // corner cases first
            if( s < 0 && t < 0 ) {s=0;t=0;}
            else if ( t < 0 && s+t>1) {s=1;t=0;}
            else if ( s < 0 && s+t>1) {s=1;t=0;} 
            else if ( s + t > 1){float z=s+t;s=s/z;t=t/z;} // then side cases
            else if ( t < 0 ){ s=s/(1-t);t=0;}
            else if ( s < 0 ){ t=t/(1-s);s=0;}
            // computed distance change
            int primid = neighbourPrims[id];
            int corners[] = primpoints(geoSp, primid);
            vector p1 = attrib(geoSp,"point","P",corners[0]);
            vector p2 = attrib(geoSp,"point","P",corners[1]);
            vector p3 = attrib(geoSp,"point","P",corners[2]);
            float improvement = length( (s - sCoords[id])*(p2-p1) + (t - tCoords[id])*(p3-p1) );
            if(improvement < minImp){
                minImp = improvement;
                chosenPrim = primid;
                chosenCoords[0]=s;chosenCoords[1]=t;
            }
            
        }
    }
}





   /*// get the attribute type name
    int attribType = attribtype(geoOr, "point", attribName );
    -1: Unknown or not found type.
    0: Integer type
    1: Float type
    2: String type
    3: Integer Array type
    4: Float Array type
    5: String Array type*/

// given a geometry and bary. coords. s,t
// to a primid with corners p1,p2,p3 in that order,
// "average" out the vector attribute between them
vector interpolationVector(int geo, chosenPrim; float s,t; string attribName){
    // now we know the 3 triangle points with weights
    // get corners of triangle
    int corners[] = primpoints(geo, chosenPrim);
    vector p1 = attrib(geo,"point",attribName,corners[0]);
    vector p2 = attrib(geo,"point",attribName,corners[1]);
    vector p3 = attrib(geo,"point",attribName,corners[2]);
    // interpolate
    return p1 + s*(p2-p1) + t*(p3-p1);    
}

// given a geometry and bary. coords. s,t
// to a primid with corners p1,p2,p3 in that order,
// "average" out the vector attribute between them
float interpolationFloat(int geo, chosenPrim; float s,t; string attribName){
    // now we know the 3 triangle points with weights
    // get corners of triangle
    int corners[] = primpoints(geo, chosenPrim);
    float p1 = attrib(geo,"point",attribName,corners[0]);
    float p2 = attrib(geo,"point",attribName,corners[1]);
    float p3 = attrib(geo,"point",attribName,corners[2]);
    // interpolate
    return p1 + s*(p2-p1) + t*(p3-p1);    
}
// given a geometry and bary. coords. s,t
// to a primid with corners p1,p2,p3 in that order,
// "average" out the vector attribute between them
int interpolationInt(int geo, chosenPrim; float s,t; string attribName){
    // now we know the 3 triangle points with weights
    // get corners of triangle
    int corners[] = primpoints(geo, chosenPrim);
    int p1 = attrib(geo,"point",attribName,corners[0]);
    int p2 = attrib(geo,"point",attribName,corners[1]);
    int p3 = attrib(geo,"point",attribName,corners[2]);
    // interpolate
    return p1 + s*(p2-p1) + t*(p3-p1);    
}

// given a geometry and bary. coords. s,t
// to a primid with corners p1,p2,p3 in that order,
// give primId and st coordinates
// return set(s,t,chosenPrim)
vector interpolationInfo(int geo, chosenPrim; float s,t; string attribName){
	return set(s,t,chosenPrim);
}


// this function will attempt to address interpolated
// values on the sphere with any spherical coordinates.
// find the related triangle on the sphere and interpolate
// its values by the neighbour points linearly.
// theta = plane, phi = north pole angle. north pole is y axis
vector interPolateSphereAttribute(int geoSp, geoOr; float theta, phi; string attribName, attribType){
    
    vector2 coords;
    int chosenPrim;
    getSTandPrimOfBarycenterInterpolation(
        geoSp, theta, phi,
        coords,
        chosenPrim);
         
    if(attribType == "vector"){    
    return interpolationVector(
            geoOr,chosenPrim,
            coords[0],coords[1],
            attribName);
    }
    if(attribType == "float"){    
    return interpolationFloat(
            geoOr,chosenPrim,
            coords[0],coords[1],
            attribName);
    }
    if(attribType == "int"){    
    return interpolationInt(
            geoOr,chosenPrim,
            coords[0],coords[1],
            attribName);
    }
	if(attribType == "info"){    
    return interpolationInt(
            geoOr,chosenPrim,
            coords[0],coords[1],
            attribName);
    }
    
}

// this function will attempt to address interpolated
// values on the sphere with any spherical coordinates.
// find the related triangle on the sphere and interpolate
// its values by the neighbour points linearly.
// theta = plane, phi = north pole angle. north pole is y axis
vector interPolateSphereAttributeVector(int geoSp, geoOr; float theta, phi; string attribName){
	// find triangle and coordinates
    vector2 coords;
    int chosenPrim;
    getSTandPrimOfBarycenterInterpolation(
        geoSp, theta, phi,
        coords,
        chosenPrim);
         // return interpolation
    return interpolationVector(
            geoOr,chosenPrim,
            coords[0],coords[1],
            attribName);
}

// this function will attempt to address interpolated
// values on the sphere with any spherical coordinates.
// find the related triangle on the sphere and interpolate
// its values by the neighbour points linearly.
// theta = plane, phi = north pole angle. north pole is y axis
float interPolateSphereAttributeFloat(int geoSp, geoOr; float theta, phi; string attribName){
	// find triangle and coordinates
    vector2 coords;
    int chosenPrim;
    getSTandPrimOfBarycenterInterpolation(
        geoSp, theta, phi,
        coords,
        chosenPrim);
         // return interpolation
    return interpolationFloat(
            geoOr,chosenPrim,
            coords[0],coords[1],
            attribName);
}			

// this function will attempt to address interpolated
// values on the sphere with any spherical coordinates.
// find the related triangle on the sphere and interpolate
// its values by the neighbour points linearly.
// theta = plane, phi = north pole angle. north pole is y axis
int interPolateSphereAttributeInt(int geoSp, geoOr; float theta, phi; string attribName){
	// find triangle and coordinates
    vector2 coords;
    int chosenPrim;
    getSTandPrimOfBarycenterInterpolation(
        geoSp, theta, phi,
        coords,
        chosenPrim);
         // return interpolation
    return interpolationInt(
            geoOr,chosenPrim,
            coords[0],coords[1],
            attribName);
}

// this function will attempt to address interpolated
// values on the sphere with any spherical coordinates.
// find the related triangle on the sphere and interpolate
// its values by the neighbour points linearly.
// theta = plane, phi = north pole angle. north pole is y axis
int interPolateSphereAttributeInfo(int geoSp, geoOr; float theta, phi; string attribName){
	// find triangle and coordinates
    vector2 coords;
    int chosenPrim;
    getSTandPrimOfBarycenterInterpolation(
        geoSp, theta, phi,
        coords,
        chosenPrim);
    // return primId
	return chosenPrim;
}

//0000000000000000000000000000000000000000000000 INTERPOLATION RELATED CODE
//0000000000000000000000000000000000000000000000 INTERPOLATION RELATED CODE
//0000000000000000000000000000000000000000000000 INTERPOLATION RELATED CODE

