module.exports = lnQuaternion;

var Vec3 = require('./Vec3');

// 'fixed' acos for inputs > 1
function acos(x) {
	// uncomment this line to cause failure for even 1/2 rotations(at the limit of the other side)
	// return Math.acos(x); // fails on rotations greater than 4pi.
	function  mod_(x,y){ return y * (x / y - Math.floor(x / y)) };
	function plusminus(x){ return mod_( x+1,2)-1 };
	function trunc(x,y){ return x-mod_(x,y) };
	return Math.acos(plusminus(x)) - trunc(x+1,2)*Math.PI/2;
}

function t(q) {
	if( isNaN( q.nx ) || isNaN( q.ny ) || isNaN( q.nz ) )
		throw new Error( "FAIL" );
//	if( isNaN( q.x ) || isNaN( q.y ) || isNaN( q.z ) )
//		throw new Error( "FAIL" );
}

/**
 * A Log Quaternion describes a rotation in 3D space. 
 * 
 * @class lnQuaternion
 * @constructor
 * @see http://en.wikipedia.org/wiki/Quaternion
 */
function lnQuaternion(){
	// axis/angle
    /**
     * @property {Number} θ
     */
	this.θ = 0;
    /**
     * @property {Number} nx 
     */
	this.nx = 0;
    /**
     * @property {Number} ny
     */
	this.ny = 1;
    /**
     * @property {Number} nz
     */
	this.nz = 0;

    /**
     * @property {Number} x
     */
    this.x = 0;

    /**
     * @property {Number} y
     */
    this.y = 0;

    /**
     * @property {Number} z
     */
    this.z = 0;
}

/**
 * Set the value of the quaternion.
 * @method set
 * @param {Number} x
 * @param {Number} y
 * @param {Number} z
 * @param {Number} w
 */
lnQuaternion.prototype.set = function(x,y,z){
    this.x = x;
    this.y = y;
    this.z = z;
	this.θ = Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z);
	if( this.θ ) {
		this.nx = this.x / this.θ;
		this.ny = this.y / this.θ;
		this.nz = this.z / this.θ;
	}else {
		this.nx = 0;
		this.ny = 1;
		this.nz = 0;
	}
t( this );
    return this;
};

/**
 * Convert to a readable format
 * @method toString
 * @return string
 */
lnQuaternion.prototype.toString = function(){
    return this.x+","+this.y+","+this.z+","+this.w;
};

/**
 * Convert to an Array
 * @method toArray
 * @return Array
 */
lnQuaternion.prototype.toArray = function(){
    return [this.x, this.y, this.z];
};

/**
 * Set the quaternion components given an axis and an angle.
 * @method setFromAxisAngle
 * @param {Vec3} axis
 * @param {Number} angle in radians
 */

lnQuaternion.prototype.getQuat = function(){
	
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
lnQuaternion.prototype.setFromAxisAngle = function(axis,angle){

	this.θ = angle;
	this.nx = axis.x;
	this.ny = axis.y;
	this.nz = axis.z;
	this.x = axis.x * angle;
	this.y = axis.y * angle;
	this.z = axis.z * angle;
t( this );

	return this;

    var s = Math.sin(angle*0.5);
    this.x = axis.x * s;
    this.y = axis.y * s;
    this.z = axis.z * s;
    this.w = Math.cos(angle*0.5);
    return this;
};

/**
 * Converts the quaternion to axis/angle representation.
 * @method toAxisAngle
 * @param {Vec3} [targetAxis] A vector object to reuse for storing the axis.
 * @return {Array} An array, first elemnt is the axis and the second is the angle in radians.
 */
lnQuaternion.prototype.toAxisAngle = function(targetAxis){
    targetAxis = targetAxis || new Vec3();
	
    targetAxis.x = this.nx; // if it is important that axis is normalised then replace with x=1; y=z=0;
    targetAxis.y = this.ny;
    targetAxis.z = this.nz;
    return [targetAxis,this.θ]
	

    this.normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
    var angle = 2 * Math.acos(this.w);
    var s = Math.sqrt(1-this.w*this.w); // assuming quaternion normalised then w is less than 1, so term always positive.
    if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
        // if s close to zero then direction of axis not important
        targetAxis.x = this.x; // if it is important that axis is normalised then replace with x=1; y=z=0;
        targetAxis.y = this.y;
        targetAxis.z = this.z;
    } else {
        targetAxis.x = this.x / s; // normalise axis
        targetAxis.y = this.y / s;
        targetAxis.z = this.z / s;
    }
    return [targetAxis,angle];
};

var sfv_t1 = new Vec3(),
    sfv_t2 = new Vec3();

/**
 * Set the quaternion value given two vectors. The resulting rotation will be the needed rotation to rotate u to v.
 * @method setFromVectors
 * @param {Vec3} u
 * @param {Vec3} v
 */
lnQuaternion.prototype.setFromVectors = function(u,v){
	console.log( "Set From Fectors is not set?" );
    if(u.isAntiparallelTo(v)){
        var t1 = sfv_t1;
        var t2 = sfv_t2;

        u.tangents(t1,t2);
        this.setFromAxisAngle(t1,Math.PI);
    } else {
        var a = u.cross(v);
			const normA = 1/Math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
		this.nx = a*normA;
		this.ny = a*normA;
		this.nz = a*normA;
		this.θ = Math.acos( Math.sqrt(Math.pow(u.norm(),2) * Math.pow(v.norm(),2)) + u.dot(v ) );
		this.x = this.nx * this.θ;
		this.y = this.ny * this.θ;
		this.z = this.nz * this.θ;
t( this );
		return this;

        this.x = a.x;
        this.y = a.y;
        this.z = a.z;
        this.w = Math.sqrt(Math.pow(u.norm(),2) * Math.pow(v.norm(),2)) + u.dot(v);
        this.normalize();
    }
    return this;
};

/**
 * lnQuaternion multiplication
 * @method mult
 * @param {lnQuaternion} q
 * @param {lnQuaternion} target Optional.
 * @return {lnQuaternion}
 */
var lnQuaternion_mult_va = new Vec3();
var lnQuaternion_mult_vb = new Vec3();
var lnQuaternion_mult_vaxvb = new Vec3();
lnQuaternion.prototype.mult = function(q,target){
	

	// q= quaternion to rotate; oct = octive to result with; ac/as cos/sin(rotation) ax/ay/az (normalized axis of rotation)
	function finishRodrigues( target, q, ax, ay, az, th ) {
		const AdotB = (q.nx*ax + q.ny*ay + q.nz*az);
	   
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
	   
			const sAng = Math.sin(ang/2);
	   
			const crsX = (ay*q.nz-az*q.ny);
			const Cx = ( crsX * cc1 +  ax * ss1 + q.nx * ss2 );
			const crsY = (az*q.nx-ax*q.nz);
			const Cy = ( crsY * cc1 +  ay * ss1 + q.ny * ss2 );
			const crsZ = (ax*q.ny-ay*q.nx);
			const Cz = ( crsZ * cc1 +  az * ss1 + q.nz * ss2 );

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
        t( target );
		return target;
	}
    target = target || new lnQuaternion();

	const ax_ = this.nx;
	const ay_ = this.ny;
	const az_ = this.nz;
	if( q.θ ) {
		// else it's unchanged
        t( target );
		return finishRodrigues( target, q, ax_, ay_, az_, this.θ );
	}else {
        target.x = q.x
        target.y = q.y
        target.z = q.z
        target.nx = q.nx
        target.ny = q.ny
        target.nz = q.nz
        target.θ = q.θ
    }
/*
	const ax_ = this.nx;
	const ay_ = this.ny;
	const az_ = this.nz;
	if( q.θ ) {
		// else it's unchanged
        t( target );
		return finishRodrigues( target, q, ax_, ay_, az_, this.θ );
	}
*/
    t( target );
	return target;



    target = target || new lnQuaternion();

    var ax = this.x, ay = this.y, az = this.z, aw = this.w,
        bx = q.x, by = q.y, bz = q.z, bw = q.w;

    target.x = ax * bw + aw * bx + ay * bz - az * by;
    target.y = ay * bw + aw * by + az * bx - ax * bz;
    target.z = az * bw + aw * bz + ax * by - ay * bx;
    target.w = aw * bw - ax * bx - ay * by - az * bz;

    return target;
};

/**
 * Get the inverse quaternion rotation.
 * @method inverse
 * @param {lnQuaternion} target
 * @return {lnQuaternion}
 */
lnQuaternion.prototype.inverse = function(target){
	return this.conjugate(target).normalize();

    var x = this.x, y = this.y, z = this.z, w = this.w;
    target = target || new lnQuaternion();

    this.conjugate(target);
    var inorm2 = 1/(x*x + y*y + z*z + w*w);
    target.x *= inorm2;
    target.y *= inorm2;
    target.z *= inorm2;
    target.w *= inorm2;

    return target;
};

/**
 * Get the quaternion conjugate
 * @method conjugate
 * @param {lnQuaternion} target
 * @return {lnQuaternion}
 */
lnQuaternion.prototype.conjugate = function(target){
    target = target || new lnQuaternion();
	target.nx = -this.nx;
	target.ny = -this.ny;
	target.nz = -this.nz;
	target.x = this.nx * this.θ;
	target.y = this.ny * this.θ;
	target.z = this.nz * this.θ;
	target.θ = this.θ;
t( target );
	return target;

    target = target || new lnQuaternion();

    target.x = -this.x;
    target.y = -this.y;
    target.z = -this.z;
    target.w = this.w;

    return target;
};

/**
 * Normalize the quaternion. Note that this changes the values of the quaternion.
 * @method normalize
 */
lnQuaternion.prototype.normalize = function(){
	// this should normalize the angle...
	this.θ %= Math.PI*2;
	this.x = this.nx * this.θ;
	this.y = this.ny * this.θ;
	this.z = this.nz * this.θ;
t(this)
	return this;

    var l = Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w);
    if ( l === 0 ) {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.w = 0;
    } else {
        l = 1 / l;
        this.x *= l;
        this.y *= l;
        this.z *= l;
        this.w *= l;
    }
    return this;
};

/**
 * Approximation of quaternion normalization. Works best when quat is already almost-normalized.
 * @method normalizeFast
 * @see http://jsperf.com/fast-quaternion-normalization
 * @author unphased, https://github.com/unphased
 */
lnQuaternion.prototype.normalizeFast = lnQuaternion.prototype.normalize

function zz() {
    var f = (3.0-(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w))/2.0;
    if ( f === 0 ) {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.w = 0;
    } else {
        this.x *= f;
        this.y *= f;
        this.z *= f;
        this.w *= f;
    }
    return this;
};

/**
 * Multiply the quaternion by a vector
 * @method vmult
 * @param {Vec3} v
 * @param {Vec3} target Optional
 * @return {Vec3}
 */
lnQuaternion.prototype.vmult = function(v,target){
    target = target || new Vec3();
    var x = v.x,
        y = v.y,
        z = v.z;


		const nst = Math.sin(this.θ/2); // normal * sin_theta
		const qw = Math.cos(this.θ/2);  //Math.cos( pl );   quaternion q.w  = (exp(lnQ)) [ *exp(lnQ.W=0) ]

		const qx = this.nx*nst;
		const qy = this.ny*nst;
		const qz = this.nz*nst;

		//p┬Æ = (v*v.dot(p) + v.cross(p)*(w))*2 + p*(w*w ┬û v.dot(v))
		const tx = 2 * (qy * v.z - qz * v.y); // v.cross(p)*w*2
		const ty = 2 * (qz * v.x - qx * v.z);
		const tz = 2 * (qx * v.y - qy * v.x);
		target.x = x + qw * tx + ( qy * tz - ty * qz );
		target.y = y + qw * ty + ( qz * tx - tz * qx );
		target.z = z + qw * tz + ( qx * ty - tx * qy );

		return target;	

{
    const qx = this.x,
        qy = this.y,
        qz = this.z,
        qw = this.w;

    // q*v
    const ix =  qw * x + qy * z - qz * y,
    iy =  qw * y + qz * x - qx * z,
    iz =  qw * z + qx * y - qy * x,
    iw = -qx * x - qy * y - qz * z;

    target.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    target.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    target.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

    return target;
}
};

/**
 * Copies value of source to this quaternion.
 * @method copy
 * @param {lnQuaternion} source
 * @return {lnQuaternion} this
 */
lnQuaternion.prototype.copy = function(source){
    this.θ = source.θ;
    this.nx = source.nx;
    this.ny = source.ny;
    this.nz = source.nz;
    this.x = source.x;
    this.y = source.y;
    this.z = source.z;
t(this);
    return this;
};

/**
 * Convert the quaternion to euler angle representation. Order: YZX, as this page describes: http://www.euclideanspace.com/maths/standards/index.htm
 * @method toEuler
 * @param {Vec3} target
 * @param string order Three-character string e.g. "YZX", which also is default.
 */
lnQuaternion.prototype.toEuler = function(target,order){
	console.log( "No TO Euler..." );
    order = order || "YZX";

    var heading, attitude, bank;
    var x = this.x, y = this.y, z = this.z, w = this.w;

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
        break;
    default:
        throw new Error("Euler order "+order+" not supported yet.");
    }

    target.y = heading;
    target.z = attitude;
    target.x = bank;
};

/**
 * See http://www.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors/content/SpinCalc.m
 * @method setFromEuler
 * @param {Number} x
 * @param {Number} y
 * @param {Number} z
 * @param {String} order The order to apply angles: 'XYZ' or 'YXZ' or any other combination
 */
lnQuaternion.prototype.setFromEuler = function ( x, y, z, order ) {
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

    } else if ( order === 'XZY' ) {

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
t(this )
    return this;
};

/**
 * @method clone
 * @return {lnQuaternion}
 */
lnQuaternion.prototype.clone = function(){
	const result = new lnQuaternion();
	result.θ = this.θ;
	result.nx = this.nx;
	result.ny = this.ny;
	result.nz = this.nz;
	result.x = this.x;
	result.y = this.y;
	result.z = this.z;
t(target )
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
lnQuaternion.prototype.slerp = function (toQuat, dt, target) {
	
	target = target || new lnQuaternion();
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
t(target )
	return target;

    var ax = this.x,
        ay = this.y,
        az = this.z,
        aw = this.w,
        bx = toQuat.x,
        by = toQuat.y,
        bz = toQuat.z,
        bw = toQuat.w;

    var omega, cosom, sinom, scale0, scale1;

    // calc cosine
    cosom = ax * bx + ay * by + az * bz + aw * bw;

    // adjust signs (if necessary)
    if ( cosom < 0.0 ) {
        cosom = -cosom;
        bx = - bx;
        by = - by;
        bz = - bz;
        bw = - bw;
    }

    // calculate coefficients
    if ( (1.0 - cosom) > 0.000001 ) {
        // standard case (slerp)
        omega  = Math.acos(cosom);
        sinom  = Math.sin(omega);
        scale0 = Math.sin((1.0 - t) * omega) / sinom;
        scale1 = Math.sin(t * omega) / sinom;
    } else {
        // "from" and "to" quaternions are very close
        //  ... so we can do a linear interpolation
        scale0 = 1.0 - t;
        scale1 = t;
    }

    // calculate final values
    target.x = scale0 * ax + scale1 * bx;
    target.y = scale0 * ay + scale1 * by;
    target.z = scale0 * az + scale1 * bz;
    target.w = scale0 * aw + scale1 * bw;

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
lnQuaternion.prototype.integrate = function(angularVelocity, dt, angularFactor, target){
    target = target || new lnQuaternion();

    target.x = this.x + angularVelocity.x * dt * angularFactor.x;
    target.y = this.y + angularVelocity.y * dt * angularFactor.y;
    target.z = this.z + angularVelocity.z * dt * angularFactor.z;

	target.θ = Math.sqrt(target.x*target.x+target.y*target.y+target.z*target.z);
	if( target.θ ) {
		target.nx = target.x / target.θ;
		target.ny = target.y / target.θ;
		target.nz = target.z / target.θ;
	}
t(target )
    return target;

    var ax = angularVelocity.x * angularFactor.x,
        ay = angularVelocity.y * angularFactor.y,
        az = angularVelocity.z * angularFactor.z,
        bx = this.x,
        by = this.y,
        bz = this.z,
        bw = this.w;

    var half_dt = dt * 0.5;

    target.x += half_dt * (ax * bw + ay * bz - az * by);
    target.y += half_dt * (ay * bw + az * bx - ax * bz);
    target.z += half_dt * (az * bw + ax * by - ay * bx);
    target.w += half_dt * (- ax * bx - ay * by - az * bz);

    return target;
};