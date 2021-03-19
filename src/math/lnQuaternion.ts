import { Vec3 } from '../math/Vec3'


// 'fixed' acos for inputs > 1
function acos(x:number):number {          
	// uncomment this line to cause failure for even 1/2 rotations(at the limit of the other side)
	// return Math.acos(x); // fails on rotations greater than 4pi.
	function  mod_(x:number,y:number):number{ return y * (x / y - Math.floor(x / y)) };
	function plusminus(x:number):number{ return mod_( x+1,2)-1 };
	function trunc(x:number,y:number):number{ return x-mod_(x,y) };
	return Math.acos(plusminus(x)) - trunc(x+1,2)*Math.PI/2;
}

/**
 * A Log Quaternion describes a rotation in 3D space. 
 * 
 * @class lnQuaternion
 * @constructor
 * @see http://en.wikipedia.org/wiki/Quaternion
 */

const sfv_t1 = new Vec3(),
    sfv_t2 = new Vec3();
const lnQuaternion_mult_va = new Vec3();
const lnQuaternion_mult_vb = new Vec3();
const lnQuaternion_mult_vaxvb = new Vec3();
const integrateTempAV = new Vec3();
const integrateTempAV2 = new Vec3();

export class Quaternion{
	// axis/angle
    /**
     * @property {Number} θ
     */
	θ = 0;
    /**
     * @property {Number} nx 
     */
	nx = 0;
    /**
     * @property {Number} ny
     */
	ny = 1;
    /**
     * @property {Number} nz
     */
	nz = 0;

    /**
     * @property {Number} x
     */
    x = 0;

    /**
     * @property {Number} y
     */
    y = 0;

    /**
     * @property {Number} z
     */
    z = 0;

	constructor() {
	}

  /*
     copy this into another quat - could be a 'get' that returns 'this'
   */
  setQuat(q:any):any { 
  	const qs:number = Math.sin( this.θ/2 );
  	const qc:number = Math.cos( this.θ/2 );
  	q.x = this.nx*qs; 
        q.y = this.ny*qs; 
        q.z = this.nz*qs; 
        q.w = qc; return q; }


/**
 * Set the value of the quaternion.
 * @method set
 * @param {Number} x
 * @param {Number} y
 * @param {Number} z
 * @param {Number} w
 */
set (x:number,y:number,z:number):Quaternion{
    x = x;
    y = y;
    z = z;
	this.θ = Math.sqrt(x*x+y*y+z*z);
	if( this.θ ) {
		this.nx = x / this.θ;
		this.ny = y / this.θ;
		this.nz = z / this.θ;
	}else {
		this.nx = 0;
		this.ny = 1;
		this.nz = 0;
	}
    return this;
};

/**
 * Convert to a readable format
 * @method toString
 * @return string
 */
toString ():string{
    return this.x+","+this.y+","+this.z;
};

/**
 * Convert to an Array
 * @method toArray
 * @return Array
 */
toArray (){
    return [this.x, this.y, this.z];
};

normalizeFast() {
}

/**
 * Set the quaternion components given an axis and an angle.
 * @method setFromAxisAngle
 * @param {Vec3} axis
 * @param {Number} angle in radians
 */

getQuat ():any{
	
	const s = Math.sin( this.θ/2 );
	const c = Math.cos( this.θ/2 );
    return {x:this.nx*s, y:this.ny*s, z:this.nz*s, w:c };
};


/**
 * Set the quaternion components given an axis and an angle.
 * @method setFromAxisAngle
 * @param {Vec3} axis
 * @param {Number} angle in radians
 */
setFromAxisAngle (axis:Vec3,angle:number){

	this.θ = angle;
	this.nx = axis.x;
	this.ny = axis.y;
	this.nz = axis.z;
	this.x = axis.x * angle;
	this.y = axis.y * angle;
	this.z = axis.z * angle;

	return this;
};

/**
 * Converts the quaternion to axis/angle representation.
 * @method toAxisAngle
 * @param {Vec3} [targetAxis] A vector object to reuse for storing the axis.
 * @return {Array} An array, first elemnt is the axis and the second is the angle in radians.
 */
toAxisAngle (targetAxis = new Vec3()):any{
	
    targetAxis.x = this.nx; // if it is important that axis is normalised then replace with x=1; y=z=0;
    targetAxis.y = this.ny;
    targetAxis.z = this.nz;
    return [targetAxis,this.θ]
	
};


/**
 * Set the quaternion value given two vectors. The resulting rotation will be the needed rotation to rotate u to v.
 * @method setFromVectors
 * @param {Vec3} u
 * @param {Vec3} v
 */
setFromVectors (u:Vec3,v:Vec3):Quaternion{
	console.log( "Set From Fectors is not set?" );
    if(u.isAntiparallelTo(v)){
        var t1 = sfv_t1;
        var t2 = sfv_t2;

        u.tangents(t1,t2);
        this.setFromAxisAngle(t1,Math.PI);
    } else {
		var a = u.cross(v);
		const normA = 1/Math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
		this.nx = a.x*normA;
		this.ny = a.y*normA;
		this.nz = a.z*normA;
		this.θ = 0;//Math.acos( Math.sqrt(Math.pow(u.norm(),2) * Math.pow(v.norm(),2)) + u.dot(v ) );
		this.x = this.nx * this.θ;
		this.y = this.ny * this.θ;
		this.z = this.nz * this.θ;
		return this;

        this.x = a.x;
        this.y = a.y;
        this.z = a.z;
        //this.w = Math.sqrt(Math.pow(u.norm(),2) * Math.pow(v.norm(),2)) + u.dot(v);
        this.normalize();
    }
    return this;
};

/**
 * lnQuaternion multiplication
 * rotates input Q around this, fills in target (which may be Q or this)
 * @method mult
 * @param {lnQuaternion} q
 * @param {lnQuaternion} target Optional.
 * @return {lnQuaternion}
 */
mult (q:Quaternion,target = new Quaternion(), dt=1):Quaternion{
	
	// q= quaternion to rotate; oct = octive to result with; ac/as cos/sin(rotation) ax/ay/az (normalized axis of rotation)
	if( q.θ ) {
		const ax = this.nx, ay = this.ny, az = this.nz, th = this.θ*dt;
		const qx = q.nx, qy = q.ny, qz = q.nz;
		const AdotB = (qx*ax + qy*ay + qz*az);
	   
		const xmy = (th - q.θ)/2; // X - Y  (x minus y)
		const xpy = (th + q.θ)/2  // X + Y  (x plus y )
		const cxmy = Math.cos(xmy);
		const cxpy = Math.cos(xpy);
		const cosCo2 = ( ( 1-AdotB )*cxmy + (1+AdotB)*cxpy )/2;
	   
		const ang = acos( cosCo2 )*2;
	   
		if( ang ) {
			const sxmy = Math.sin(xmy);
			const sxpy = Math.sin(xpy);
	   
			const ss1 = sxmy + sxpy
			const ss2 = sxpy - sxmy
			const cc1 = cxmy - cxpy
	   
			const crsX = (ay*qz-az*qy);
			const Cx = ( crsX * cc1 +  ax * ss1 + qx * ss2 );
			const crsY = (az*qx-ax*qz);
			const Cy = ( crsY * cc1 +  ay * ss1 + qy * ss2 );
			const crsZ = (ax*qy-ay*qx);
			const Cz = ( crsZ * cc1 +  az * ss1 + qz * ss2 );

			const Clx = 1/Math.sqrt(Cx*Cx+Cy*Cy+Cz*Cz);
	   
			target.θ  = ang;
			target.nx = Cx*Clx;
			target.ny = Cy*Clx;
			target.nz = Cz*Clx;
	   
			target.x  = target.nx*ang;
			target.y  = target.ny*ang;
			target.z  = target.nz*ang;
	   
		} else {
			// two axles are coincident, add...
			if( AdotB > 0 ) {
				target.θ += th;
				target.x = target.nx * (target.θ);
				target.y = target.ny * (target.θ);
				target.z = target.nz * (target.θ);
			}else {
				target.θ -= th;
				target.x = target.nx * (target.θ);
				target.y = target.ny * (target.θ);
				target.z = target.nz * (target.θ);
			}
		}
		return target;
	}else {
        target.x = this.x
        target.y = this.y
        target.z = this.z
        target.nx = this.nx
        target.ny = this.ny
        target.nz = this.nz
        target.θ = this.θ
    }
	return target;
};

/**
 * Get the inverse quaternion rotation.
 * @method inverse
 * @param {lnQuaternion} target
 * @return {lnQuaternion}
 */
inverse (target:Quaternion):Quaternion{
	return this.conjugate(target).normalize();
};

/**
 * Get the quaternion conjugate
 * @method conjugate
 * @param {lnQuaternion} target
 * @return {lnQuaternion}
 */
conjugate (target = new Quaternion()):Quaternion{
    target = this;
	target.nx = -this.nx;
	target.ny = -this.ny;
	target.nz = -this.nz;
	target.x = this.nx * this.θ;
	target.y = this.ny * this.θ;
	target.z = this.nz * this.θ;
//	target.θ = this.θ;
	return target;
};

/**
 * Normalize the quaternion. Note that this changes the values of the quaternion.
 * @method normalize
 */
normalize ():Quaternion{
	// this should normalize the angle...
    /*
	θ %= Math.PI*2;
	x = this.nx * this.θ;
	y = this.ny * this.θ;
	z = this.nz * this.θ;
    */
	return this;
};

/**
 * Multiply the quaternion by a vector
 * @method vmult
 * @param {Vec3} v
 * @param {Vec3} target Optional
 * @return {Vec3}
 */
vmult (v:Vec3,target = new Vec3(), dt?:any):Vec3{
	dt = dt||1
	const c = Math.cos(this.θ*dt);//
	const s = Math.sin(this.θ*dt);//

	const qx = this.nx, qy = this.ny, qz = this.nz;
	const vx = v.x , vy = v.y , vz = v.z;
	// sin theta * cross
	// 19 mult 12 add
	const cx =  s*(qy * vz - qz * vy);
	const cy =  s*(qz * vx - qx * vz);
	const cz =  s*(qx * vy - qy * vx);
	// (1-cos theta) * dot
	const dot =  (1-c)*((qx * vx ) + (qy*vy)+(qz*vz));
	// v *cos(theta) + cross*sin(theta) + this * dot * (1-c)
	target.x = vx*c + cx + qx * dot;
	target.y = vy*c + cy + qy * dot
	target.z = vz*c + cz + qz * dot;

	return target;	
};

/**
 * Copies value of source to this quaternion.
 * @method copy
 * @param {lnQuaternion} source
 * @return {lnQuaternion} this
 */
copy (source:Quaternion):Quaternion{
    this.θ = source.θ;
    this.nx = source.nx;
    this.ny = source.ny;
    this.nz = source.nz;
    this.x = source.x;
    this.y = source.y;
    this.z = source.z;
    return this;
};

/**
 * Convert the quaternion to euler angle representation. Order: YZX, as this page describes: http://www.euclideanspace.com/maths/standards/index.htm
 * @method toEuler
 * @param {Vec3} target
 * @param string order Three-character string e.g. "YZX", which also is default.
 */
toEuler (target:Vec3,order:string){
	console.log( "No TO Euler..." );
    order = order || "YZX";

    var heading = NaN, attitude, bank;
    var x = this.x, y = this.y, z = this.z, w = Math.cos(this.θ/2);

    switch(order){
    case "YZX":
        var test = x*y + z*w;
        if (test > 0.499) { // singularity at north pole
            heading = 2 * Math.atan2(x,w);
            attitude = Math.PI/2;
            bank = 0;
        }
        if (test < -0.499) { // singularity at south pole
            heading = -2 * Math.atan2(x,w);
            attitude = - Math.PI/2;
            bank = 0;
        }
        if(isNaN(heading)){
            var sqx = x*x;
            var sqy = y*y;
            var sqz = z*z;
            heading = Math.atan2(2*y*w - 2*x*z , 1 - 2*sqy - 2*sqz); // Heading
            attitude = Math.asin(2*test); // attitude
            bank = Math.atan2(2*x*w - 2*y*z , 1 - 2*sqx - 2*sqz); // bank
        }
    target.y = heading || 0;
    target.z = attitude || 0;
    target.x = bank || 0;
        break;
    default:
        throw new Error("Euler order "+order+" not supported yet.");
    }

};

/**
 * See http://www.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/content/SpinCalc.m
 * @method setFromEuler
 * @param {Number} x
 * @param {Number} y
 * @param {Number} z
 * @param {String} order The order to apply angles: 'XYZ' or 'YXZ' or any other combination
 */
setFromEuler  ( x:number, y:number, z:number, order:string ):Quaternion {
    order = order || "XYZ";

    var c1 = Math.cos( x / 2 );
    var c2 = Math.cos( y / 2 );
    var c3 = Math.cos( z / 2 );
    var s1 = Math.sin( x / 2 );
    var s2 = Math.sin( y / 2 );
    var s3 = Math.sin( z / 2 );
    var tx, ty, tz, tw;
    if ( order === 'XYZ' ) {

        tx = s1 * c2 * c3 + c1 * s2 * s3;
        ty = c1 * s2 * c3 - s1 * c2 * s3;
        tz = c1 * c2 * s3 + s1 * s2 * c3;
        tw = c1 * c2 * c3 - s1 * s2 * s3;

    } else if ( order === 'YXZ' ) {

        tx = s1 * c2 * c3 + c1 * s2 * s3;
        ty = c1 * s2 * c3 - s1 * c2 * s3;
        tz = c1 * c2 * s3 - s1 * s2 * c3;
        tw = c1 * c2 * c3 + s1 * s2 * s3;

    } else if ( order === 'ZXY' ) {

        tx = s1 * c2 * c3 - c1 * s2 * s3;
        ty = c1 * s2 * c3 + s1 * c2 * s3;
        tz = c1 * c2 * s3 + s1 * s2 * c3;
        tw = c1 * c2 * c3 - s1 * s2 * s3;

    } else if ( order === 'ZYX' ) {

        tx = s1 * c2 * c3 - c1 * s2 * s3;
        ty = c1 * s2 * c3 + s1 * c2 * s3;
        tz = c1 * c2 * s3 - s1 * s2 * c3;
        tw = c1 * c2 * c3 + s1 * s2 * s3;

    } else if ( order === 'YZX' ) {

        tx = s1 * c2 * c3 + c1 * s2 * s3;
        ty = c1 * s2 * c3 + s1 * c2 * s3;
        tz = c1 * c2 * s3 - s1 * s2 * c3;
        tw = c1 * c2 * c3 - s1 * s2 * s3;

    } else /*if ( order === 'XZY' )*/ {

        tx = s1 * c2 * c3 - c1 * s2 * s3;
        ty = c1 * s2 * c3 - s1 * c2 * s3;
        tz = c1 * c2 * s3 + s1 * s2 * c3;
        tw = c1 * c2 * c3 + s1 * s2 * s3;
    }

	this.θ = Math.acos( tw ) *2;
	const sinTheta = Math.sin( this.θ / 2 );
	this.nx = tx / sinTheta;
	this.ny = ty / sinTheta;
	this.nz = tz / sinTheta;
	this.x = this.nx * this.θ;
	this.y = this.ny * this.θ;
	this.z = this.nz * this.θ;
    return this;
};

/**
 * @method clone
 * @return {lnQuaternion}
 */
clone ():Quaternion{
	const result = new Quaternion();
	result.θ = this.θ;
	result.nx = this.nx;
	result.ny = this.ny;
	result.nz = this.nz;
	result.x = this.x;
	result.y = this.y;
	result.z = this.z;
    return result;
};

/**
 * Performs a spherical linear interpolation between two quat
 *
 * @method slerp
 * @param {lnQuaternion} tolnQuat second operand
 * @param {Number} t interpolation amount between the self quaternion and tolnQuat
 * @param {lnQuaternion} [target] A quaternion to store the result in. If not provided, a new one will be created.
 * @returns {lnQuaternion} The "target" object
 */
slerp  (toQuat:Quaternion, dt:number, target = new Quaternion()):Quaternion {
	
	target.x = this.x * (1-dt) + toQuat.x * dt;
	target.y = this.y * (1-dt) + toQuat.y * dt;
	target.z = this.z * (1-dt) + toQuat.z * dt;
	target.θ = Math.sqrt(target.x*target.x+target.y*target.y+target.z*target.z);
	if( target.θ ) {
		target.nx = target.x / target.θ;
		target.ny = target.y / target.θ;
		target.nz = target.z / target.θ;
	}else {
		target.nx = 0;
		target.ny = 1;
		target.nz = 0;
	}
	return target;
};

/**
 * Rotate an absolute orientation quaternion given an angular velocity and a time step.
 * @param  {Vec3} angularVelocity
 * @param  {number} dt
 * @param  {Vec3} angularFactor
 * @param  {lnQuaternion} target
 * @return {lnQuaternion} The "target" object
 */
integrate (angularVelocity:Vec3, dt:number, angularFactor:Vec3, target = new Quaternion()):Quaternion{
	if(1) {

		// attempt convert to quaternions, do the math, and reverse (fails)
		const s = Math.sin(this.θ/2);
		var ax = angularVelocity.x * angularFactor.x,
			ay = angularVelocity.y * angularFactor.y,
			az = angularVelocity.z * angularFactor.z,
			bx = this.nx * s,
			by = this.ny * s,
			bz = this.nz * s,
			bw = Math.cos( this.θ/2);

		const ts = Math.sin(target.θ/2);
		// target is assumed to already be valid... this is adding to target.
		var tx = target.nx * ts;
		var ty = target.ny * ts;
		var tz = target.nz * ts;
		var tw = Math.cos(target.θ/2);   // not used in output

		var half_dt = dt * 0.5;

		tx += half_dt * (ax * bw + ay * bz - az * by);
		ty += half_dt * (ay * bw + az * bx - ax * bz);
		tz += half_dt * (az * bw + ax * by - ay * bx);
		tw += half_dt * (- ax * bx - ay * by - az * bz);   // not used in output
		const qlen = Math.sqrt(tw*tw+tx*tx+ty*ty+tz*tz);
		if( qlen ) {
			tw /= qlen;
			tx /= qlen;
			ty /= qlen;
			tz /= qlen;
		}
		const len = Math.sqrt(tx*tx+ty*ty+tz*tz);
		// length may be greater than 1; and sin returns NaN for len>1
		if( len ) {
			target.θ = Math.asin( len)*2;
			target.nx = tx / len;
			target.ny = ty / len;
			target.nz = tz / len;
		}else {
			target.nx = 0;
			target.ny = 0;
			target.nz = 1;
		}

		target.x = target.nx * 	target.θ;
		target.y = target.ny * 	target.θ;
		target.z = target.nz * 	target.θ;

		return target;
	}

	if(0)
	{
		// try to make multiple steps...
		const step = 1/20;
		
		if( target != this ) {
			// innitialize target to this.
			target.x = this.x;
			target.y = this.y;
			target.z = this.z;
			target.nx = this.nx;
			target.ny = this.ny;
			target.nz = this.nz;
			target.θ = this.θ;
		}
		
		integrateTempAV.x = angularVelocity.x *dt*angularFactor.x;
		integrateTempAV.y = angularVelocity.y *dt*angularFactor.y;
		integrateTempAV.z = angularVelocity.z *dt*angularFactor.z;
		const len = Math.sqrt(integrateTempAV.x*integrateTempAV.x+integrateTempAV.y*integrateTempAV.y+integrateTempAV.z*integrateTempAV.z);
		// length of this vector is already scaled by dt... 
		for( let t = 0; t < 1; t+=step ) {
			target.vmult( integrateTempAV, integrateTempAV2, -1 );
			
			/*
			target.x += integrateTempAV2.x*step;
			target.y += integrateTempAV2.y*step;
			target.z += integrateTempAV2.z*step;
			target.θ = Math.sqrt(target.x*target.x+target.y*target.y+target.z*target.z);
			if( target.θ ) {
				target.nx = target.x / target.θ;
				target.ny = target.y / target.θ;
				target.nz = target.z / target.θ;
			}
			*/
			
			integrateTempAVQ.x = integrateTempAV2.x;
			integrateTempAVQ.y = integrateTempAV2.y;
			integrateTempAVQ.z = integrateTempAV2.z;
			integrateTempAVQ.θ = len;
			if( len ) {
				integrateTempAVQ.nx = integrateTempAVQ.x / integrateTempAVQ.θ;
				integrateTempAVQ.ny = integrateTempAVQ.y / integrateTempAVQ.θ;
				integrateTempAVQ.nz = integrateTempAVQ.z / integrateTempAVQ.θ;
			}

			// step target by the updated axis-angle rotation... 
			integrateTempAVQ.mult( target, target, step );
			
		}
	
		return target;
	}

	if( 1) {
		// most working for boxes settling on plane
		// point-to-point constraint gains momentum over time.
		const thisx = this.x;
		const thisy = this.y;
		const thisz = this.z;
		
		integrateTempAV.x = angularVelocity.x *dt*angularFactor.x;
		integrateTempAV.y = angularVelocity.y *dt*angularFactor.y;
		integrateTempAV.z = angularVelocity.z *dt*angularFactor.z;

		this.vmult( integrateTempAV, integrateTempAV, -0.5 );
		if( 1 ) {			
			target.x = integrateTempAV.x + thisx;
			target.y = integrateTempAV.y + thisy;
			target.z = integrateTempAV.z + thisz;

			target.θ = Math.sqrt(target.x*target.x+target.y*target.y+target.z*target.z);

			if( target.θ ) {
				target.nx = target.x / target.θ;
				target.ny = target.y / target.θ;
				target.nz = target.z / target.θ;
				// normalize fast.
				if( target.θ > Math.PI*4 )  { // 0 to 2pi
					target.θ %= Math.PI*4;
					target.x = target.nx * target.θ;
					target.y = target.ny * target.θ;
					target.z = target.nz * target.θ;
				}
			}else {
				target.nx = 0;
				target.ny = 1;
				target.nz = 0;
			}
		}	
		return target;


		if( 0 ) {
			// convert vector to a rotation vector, then rotate
			// 'this' by that.
			integrateTempAVQ.x = integrateTempAV.x
			integrateTempAVQ.y = integrateTempAV.y
			integrateTempAVQ.z = integrateTempAV.z
			integrateTempAVQ.θ = Math.sqrt(integrateTempAVQ.x*integrateTempAVQ.x
				+integrateTempAVQ.y*integrateTempAVQ.y
				+integrateTempAVQ.z*integrateTempAVQ.z);
			if( integrateTempAVQ.θ) {
				integrateTempAVQ.nx =integrateTempAV.x / integrateTempAVQ.θ;
				integrateTempAVQ.ny =integrateTempAV.y / integrateTempAVQ.θ;
				integrateTempAVQ.nz =integrateTempAV.z / integrateTempAVQ.θ;
			}else {
				integrateTempAVQ.nx =0;
				integrateTempAVQ.ny =0;
				integrateTempAVQ.nz =1;
	
			}
			integrateTempAVQ.mult( this, target );
		}
	}




	if(0) {
        // simplest, most direct idea (fails for idle boxes on the plane)
		const thisx = this.x;
		const thisy = this.y;
		const thisz = this.z;
		
		target.x = this.x + angularVelocity.x *dt*angularFactor.x;
		target.y = this.y + angularVelocity.y *dt*angularFactor.y;
		target.z = this.z + angularVelocity.z *dt*angularFactor.z;

		target.θ = Math.sqrt(target.x*target.x+target.y*target.y+target.z*target.z);

		if( target.θ ) {
			target.nx = target.x / target.θ;
			target.ny = target.y / target.θ;
			target.nz = target.z / target.θ;
			// normalize fast.
			if( target.θ > Math.PI*2 )  { // 0 to 2pi
				target.θ %= Math.PI*2;
				target.x = target.nx * target.θ;
				target.y = target.ny * target.θ;
				target.z = target.nz * target.θ;
			}
		}else {
			target.nx = 0;
			target.ny = 1;
			target.nz = 0;
		}
	}
	return target;
};

}

const integrateTempAVQ = new Quaternion();
const integrateTempAVQ2 = new Quaternion();
