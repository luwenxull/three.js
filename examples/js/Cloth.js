/*
 * Cloth Simulation using a relaxed constraints solver
 */

// Suggested Readings

// Advanced Character Physics by Thomas Jakobsen Character
// http://freespace.virgin.net/hugo.elias/models/m_cloth.htm
// http://en.wikipedia.org/wiki/Cloth_modeling
// http://cg.alexandra.dk/tag/spring-mass-system/
// Real-time Cloth Animation http://www.darwin3d.com/gamedev/articles/col0599.pdf

var DAMPING = 0.03;
var DRAG = 1 - DAMPING; // 阻尼
var MASS = 0.1;
var restDistance = 25;

var xSegs = 10;
var ySegs = 10;

var clothFunction = plane( restDistance * xSegs, restDistance * ySegs );

var cloth = new Cloth( xSegs, ySegs );

var GRAVITY = 981 * 1.4;
// 重力
var gravity = new THREE.Vector3( 0, - GRAVITY, 0 ).multiplyScalar( MASS );


var TIMESTEP = 18 / 1000;
var TIMESTEP_SQ = TIMESTEP * TIMESTEP;

var pins = [];


var wind = true;
var windStrength = 2; // 风力
var windForce = new THREE.Vector3( 0, 0, 0 ); // 风

var ballPosition = new THREE.Vector3( 0, - 45, 0 );
var ballSize = 60; //40

var tmpForce = new THREE.Vector3();

var lastTime;


function plane( width, height ) {

	return function ( u, v, target ) {
  /**
   * clothFunction
   * 用来生成衣物对应的geometry(parametricGeometry)
   * u,v范围: [0, 1]
   */

		var x = ( u - 0.5 ) * width;
		var y = ( v + 0.5 ) * height;
		var z = 0;

		target.set( x, y, z );

	};

}

/**
 * 衣服粒子
 * 每个粒子对应geometry的vector
 * @param x
 * @param y
 * @param z
 * @param mass 质量
 * @constructor
 */
function Particle( x, y, z, mass ) {

	this.position = new THREE.Vector3();
	this.previous = new THREE.Vector3();
	this.original = new THREE.Vector3();
	this.a = new THREE.Vector3( 0, 0, 0 ); // acceleration
	this.mass = mass;
	this.invMass = 1 / mass;
	this.tmp = new THREE.Vector3(); // 位置
	this.tmp2 = new THREE.Vector3(); // 加速度

	// init

	clothFunction( x, y, this.position ); // position
	clothFunction( x, y, this.previous ); // previous
	clothFunction( x, y, this.original );

}

// Force -> Acceleration

// 力转成加速度
Particle.prototype.addForce = function ( force ) {

	this.a.add(
		this.tmp2.copy( force ).multiplyScalar( this.invMass )
	);

};


// Performs Verlet integration

Particle.prototype.integrate = function ( timesq ) {

	var newPos = this.tmp.subVectors( this.position, this.previous ); // 上一帧的位置指向当前位置
	newPos.multiplyScalar( DRAG ).add( this.position ); // 计算惯性与阻尼的影响
	newPos.add( this.a.multiplyScalar( timesq ) );

	this.tmp = this.previous;
	this.previous = this.position;
	this.position = newPos;

	this.a.set( 0, 0, 0 ); // 加速度归零

};


var diff = new THREE.Vector3();

function satisfyConstraints( p1, p2, distance ) {

	diff.subVectors( p2.position, p1.position ); // p1指向p2
	var currentDist = diff.length();
	if ( currentDist === 0 ) return; // prevents division by 0
	var correction = diff.multiplyScalar( 1 - distance / currentDist );
	var correctionHalf = correction.multiplyScalar( 0.5 );
	p1.position.add( correctionHalf );
	p2.position.sub( correctionHalf );

}


function Cloth( w, h ) {

	w = w || 10;
	h = h || 10;
	this.w = w;
	this.h = h;

	var particles = [];
	var constraints = [];

	var u, v;

	// Create particles
  // 注意！
	// 行和列都分成了10份，所以每行或每列的个数是11
	// 后续构建约束的时候需要注意
	for ( v = 0; v <= h; v ++ ) {

		for ( u = 0; u <= w; u ++ ) {

			particles.push(
				new Particle( u / w, v / h, 0, MASS )
			);

		}

	}

	// Structural
	// 约束
	for ( v = 0; v < h; v ++ ) {

		for ( u = 0; u < w; u ++ ) {

			constraints.push( [
				particles[ index( u, v ) ],
				particles[ index( u, v + 1 ) ], // 下
				restDistance
			] );

			constraints.push( [
				particles[ index( u, v ) ],
				particles[ index( u + 1, v ) ], // 右
				restDistance
			] );

		}

	}

	// 边界的特殊处理
	for ( u = w, v = 0; v < h; v ++ ) {

		constraints.push( [
			particles[ index( u, v ) ],
			particles[ index( u, v + 1 ) ],
			restDistance

		] );

	}

	for ( v = h, u = 0; u < w; u ++ ) {

		constraints.push( [
			particles[ index( u, v ) ],
			particles[ index( u + 1, v ) ],
			restDistance
		] );

	}


	// While many systems use shear and bend springs,
	// the relaxed constraints model seems to be just fine
	// using structural springs.
	// Shear
	// var diagonalDist = Math.sqrt(restDistance * restDistance * 2);


	// for (v=0;v<h;v++) {
	// 	for (u=0;u<w;u++) {

	// 		constraints.push([
	// 			particles[index(u, v)],
	// 			particles[index(u+1, v+1)],
	// 			diagonalDist
	// 		]);

	// 		constraints.push([
	// 			particles[index(u+1, v)],
	// 			particles[index(u, v+1)],
	// 			diagonalDist
	// 		]);

	// 	}
	// }


	this.particles = particles;
	this.constraints = constraints;

  /**
   *
   * @param u
   * @param v
   * @return {*}
   */
	function index( u, v ) {

		return u + v * ( w + 1 ); // 分割成w端，所以每行有w+1个粒子

	}

	this.index = index;

}

function simulate( time ) {

	if ( ! lastTime ) {

		lastTime = time;
		return;

	}

	var i, il, particles, particle, pt, constraints, constraint;

	// Aerodynamics forces

	if ( wind ) {

		var face, faces = clothGeometry.faces, normal;

		particles = cloth.particles;

		for ( i = 0, il = faces.length; i < il; i ++ ) {

			face = faces[ i ];
			normal = face.normal;

			tmpForce.copy( normal ).normalize().multiplyScalar( normal.dot( windForce ) /* 风向分量 */ );
			particles[ face.a ].addForce( tmpForce );
			particles[ face.b ].addForce( tmpForce );
			particles[ face.c ].addForce( tmpForce );

		}

	}

	for ( particles = cloth.particles, i = 0, il = particles.length; i < il; i ++ ) {

		particle = particles[ i ];
		particle.addForce( gravity ); // 添加重力

		particle.integrate( TIMESTEP_SQ );

	}

	// Start Constraints

	constraints = cloth.constraints;
	il = constraints.length;

	for ( i = 0; i < il; i ++ ) {

		constraint = constraints[ i ];
		satisfyConstraints( constraint[ 0 ], constraint[ 1 ], constraint[ 2 ] );

	}

	// Ball Constraints

	ballPosition.z = - Math.sin( Date.now() / 600 ) * 90; //+ 40;
	ballPosition.x = Math.cos( Date.now() / 400 ) * 70;

	if ( sphere.visible ) {

		for ( particles = cloth.particles, i = 0, il = particles.length; i < il; i ++ ) {

			particle = particles[ i ];
			var pos = particle.position;
			diff.subVectors( pos, ballPosition );
			if ( diff.length() < ballSize ) {

				// collided
				diff.normalize().multiplyScalar( ballSize );
				pos.copy( ballPosition ).add( diff );

			}

		}

	}


	// Floor Constraints

	for ( particles = cloth.particles, i = 0, il = particles.length; i < il; i ++ ) {

		particle = particles[ i ];
		pos = particle.position;
		if ( pos.y < - 250 ) {

			pos.y = - 250;

		}

	}

	// Pin Constraints

	for ( i = 0, il = pins.length; i < il; i ++ ) {

		var xy = pins[ i ];
		var p = particles[ xy ];
		p.position.copy( p.original );
		p.previous.copy( p.original );

	}


}
