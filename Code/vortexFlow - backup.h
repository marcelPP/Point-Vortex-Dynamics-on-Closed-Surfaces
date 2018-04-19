#include "math.h"
#include "$HIP/Code/Sphere.h"
// Use this to compute velocites on vortex dynamics

// define general velocity function
void vortexVelocity(
    vector P; // position of particle
    vector N; // normal of particle
    int isVortex; // avoid self advection/ allow passive particles
    int vortexId; // only relevant if it is vortex
    int nrOfVortices; // @numpt in vortex only system
    int geo; // geo input from which vortices are from. vortex only -> 0
    string positionOfVortexAttributeName; // "P" at start, then the advected one for RK4
    string geometryType; // "plane", "sphere", "general"
    export vector velocity){
    float pi = 3.14159265359;    

    // attrtribute for velocity of each vortex. Reset here.
    velocity = set(0,0,0);
    int geoSp = 1; // geometry of sphere in general case
    
    // current normal
    //vector N = attrib(geo,'point','N',pointId);
    // position of vertexId
    //vector P = attrib(geo,'point','P',pointId);
    
    
    // advect each vertex through the forces of all other vertices
    for( int forceId = 0; forceId < nrOfVortices; forceId++){
    
        vector vortexP = attrib(geo,"point",positionOfVortexAttributeName,forceId);
        vector edge = P - vortexP;
        float dist = length(edge);
    
        // skip self force if vortex
        if (isVortex && dist<pow(10,-10) ){ continue; }
        if (isVortex && vortexId==forceId ){ continue; }
        
        // get vorticity
        float vorticity = attrib(geo,"point","vorticity",forceId);
        
        // compute force exerted on vertexId from forceId
        // distinguish geometric case
        if( geometryType == "plane"){
            // using the formula from the cclin planar vortex source
            velocity += vorticity/(2*pi)*cross(N,edge)/dist/dist;
            
        } else if (geometryType == "sphere"){
            // based on boato
            velocity += vorticity/(4*pi)*cross(vortexP,P)/(1-dot(vortexP,P));
    
        } else if(geometryType == "sphere_Marcel"){
            // based on my first attempt
            // take the spherical distance
            dist = lengthSphere(P,vortexP);
            // and project the raw edge to the tangen plane and preserve the lengt
            // this is the parallel transport
            vector rawEdge = edge;
            edge = rawEdge - dot(P,rawEdge)*P;
            edge = edge/length(edge)*dist; //length(edge)=dist
            velocity += vorticity/(2*pi)*cross(N,edge)/dist/dist;

    
        } else {// do general magic
            // take the geometry of the conformal sphere map
            // and do modified sphere advection there
            // according to Boatto & Koiller
            // recall: G=h^2*g conform factor
            
            // compute sphere velocity by vortex
            vector sphereVel = cross(vortexP,P)/(1-dot(vortexP,P));                  
            float fac = vorticity/(4*pi);
            velocity += fac*sphereVel;

            // get conformal grad
            // get info what prim we are on
            int primId = interPolateSphereAttributeInfo(
                geoSp, geoSp,
                theta(P), phi(P),
                "-");
            vector gradh = attrib(geoSp,"prim","gradh",primId);
        
            /*// get linear comb of conformal grad
            vector gradh = interPolateSphereAttributeVector(
            geoSp, geoSp,
            theta(P), phi(P),
            "areaAvgGradh");*/
			
			// get linear comb of conformal factor
			float h = interPolateSphereAttributeFloat(
            geoSp, geoSp,
            theta(P), phi(P),
            "h");
		
		
            // curvature velocity 
			//vector confVel = 0.5*cross(P,gradh); 
            vector confVel = cross(P,gradh)/h; 
            //confVel = set(0,0,0); //constant test 
            //confVel = gradh/length(gradh)/10; // grad acent test
            //velocity = confVel;
            velocity += fac*confVel;
            
        }
        
    } // endfor forceId 
    

    // only added once per vortex
    if(geometryType == "general"){
    
        // grab conformal factor
        // get linear comb of conformal factor
        float h = interPolateSphereAttributeFloat(
            geoSp, geoSp,
            theta(P), phi(P),
            "h");
            
        // now finally scale the velocity correctly
        velocity/=(h*h);
    }
    
}



// advect distance of velo in that direction but
// in the spherical distance sense
// returns the new position of the advected point
vector sphereAdvect(
        vector pos; // position
        vector velo;
        float dt){
        // normal advection
        vector next = pos + velo*dt;
        // advection in sphere coordinates
        vector axis = cross(pos,next);
        vector4 Q = quaternion(
                length(velo)*dt,
                axis/length(axis));
        pos = qrotate(Q,pos);
        return pos/length(pos); // just to be sure
        
}  


