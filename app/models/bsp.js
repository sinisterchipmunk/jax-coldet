var BSP = (function() {
  // +level+ is an array, containing either Triangles or BSP nodes.
  // This function replaces every 2 elements in the array with a single
  // parent BSP node. The array is modified in-place and the size of the
  // array should be cut in half, to a minimum of 1.
  function buildLevel(level) {
    var nextLevel = [];
    var plane = new Jax.Geometry.Plane();
    while (level.length > 0) {
      var front = level.shift(), back = null;
      var dist = vec3.create();
      var closest = null, closest_index;
      
      var result = front;
      if (level.length > 0) {
        for (var j = 0; j < level.length; j++) {
          var len = vec3.length(vec3.subtract(front.center, level[j].center, dist));
          if (closest == null || closest > len) {
            closest = len;
            closest_index = j;
          }
        }
        back = level[closest_index];
        level.splice(closest_index, 1);

        // See if back and front are accurate. If not, swap them.

        // If triangle, use the plane created by the current triangle.
        // If node, use the first triangle in the box for a plane.
        if (front instanceof Jax.Geometry.Triangle)
          plane.set(front.a, front.b, front.c);
        else {
          var tri = front.front;
          while (tri instanceof BSP) tri = tri.front;
          plane.set(tri.a, tri.b, tri.c);
        }
        
        if (plane.whereis(back.center) == Jax.Geometry.Plane.FRONT)
          result = new BSP(back, front);
        else result = new BSP(front, back);
      }
      
      nextLevel.push(result);
    }
    
    for (var i = 0; i < nextLevel.length; i++)
      level.push(nextLevel[i]);
  }
  
  // Calculates the dimensions of a bounding box around
  // the given Triangle. If the triangle is axis-aligned,
  // one of the dimensions will be 0; in this case, 
  // that dimension of the bounding box will be set to a
  // very small positive value, instead.
  function calcTriangleDimensions(tri) {
    var result = vec3.create();
    var min = vec3.create(), max = vec3.create();

    vec3.min(vec3.min(tri.a, tri.b, min), tri.c, min);
    vec3.max(vec3.max(tri.a, tri.b, max), tri.c, max);
    min_size = Math.EPSILON * 2;
    for (var i = 0; i < 3; i++) {
      result[i] = max[i] - min[i];
      if (result[i] < min_size) result[i] = min_size;
    }

    return vec3.scale(result, 0.5);
  }
  
  return Jax.Class.create({
    initialize: function(front, back) {
      this.front = null;
      this.back = null;
      this.triangles = [];
      if (front || back) this.set(front, back);
    },
    
    /**
     * BSP#getClosestNode(point) -> BSP | Jax.Geometry.Triangle
     * - point (vec3): the point to be tested
     * 
     * Returns the node closest to the given point. If only one
     * sub-node exists, that node is returned. If no sub-nodes
     * exist, this node is returned.
     **/
    getClosestNode: function(point) {
      if (!this.front || !this.back) return this.front || this.back || this;
      var vec = vec3.create();
      var dist = vec3.length(vec3.subtract(this.front.center, point, vec));
      if (dist < vec3.length(vec3.subtract(this.back.center, point, vec)))
        return this.front;
      return this.back;
    },
    
    set: function(nodeFront, nodeBack) {
      this.front = nodeFront;
      this.back = nodeBack;
      this.center = vec3.create();
      
      var c = 0;
      if (nodeFront)  { vec3.add(this.center, nodeFront.center,  this.center); c++; }
      if (nodeBack)   { vec3.add(this.center, nodeBack.center, this.center); c++; }
      if (c > 0) vec3.scale(this.center, 1.0 / c);
      
      var halfSize = this.calcHalfSize();
      this.box = new Box(vec3.subtract(this.center, halfSize, vec3.create()),
                         vec3.scale(halfSize, 2, vec3.create()));
    },
    
    /**
     * BSP#collide(other, transform) -> Boolean | Object
     * - other (BSP): the potentially-colliding BSP model
     * - transform (mat4): a transformation matrix which is used to convert
     *                     +other+ into this BSP's coordinate space.
     *
     * Applies the given transformation matrix to +other+; if any triangle
     * within +other+ is intersecting any triangle in this BSP tree, then
     * a generic object containing the properties +first+, +second+ and
     * +second_transformed+ is returned. They have the following meanings:
     *
     * * +first+ : the colliding triangle in this BSP tree
     * * +second+: the colliding triangle in the +other+ BSP tree
     * * +second_transformed+: a copy of the colliding triangle in the +other+
     *                         BSP tree, transformed by the matrix to be
     *                         in this BSP tree’s coordinate space.
     *
     * If no collision has occurred, +false+ is returned.
     *
     **/
    collide: function(other, transform) {
      if (!this.finalized) this.finalize();
      if (!other.finalized) other.finalize();
      
      // buffer checks for GC optimization
      var checks = this.checks = this.checks || [{}];
      var check_id = 1;
      checks[0][0] = this;
      checks[0][1] = other;
      var tri = new Jax.Geometry.Triangle(), a = vec3.create(), b = vec3.create(), c = vec3.create();
      
      while (check_id > 0) {
        var check = checks[--check_id];
        var first = check[0], second = check[1];
        if (first instanceof BSP && second instanceof BSP) {
          // both elements are nodes, if they intersect move to the next level;
          // if they don't intersect, let them disappear.
          if (first.box.intersectOBB(second.box, transform)) {
            while (checks.length - check_id < 4) checks.push([{}]);
            checks[check_id  ][0] = first.front;  checks[check_id  ][1] = second.front;
            checks[check_id+1][0] = first.back;   checks[check_id+1][1] = second.front;
            checks[check_id+2][0] = first.front;  checks[check_id+2][1] = second.back;
            checks[check_id+3][0] = first.back;   checks[check_id+3][1] = second.back;
            check_id += 4;
          }
        } else if (first instanceof Jax.Geometry.Triangle && second instanceof BSP) {
          // front is a tri, keep it to retest against back's children
          while (checks.length - check_id < 2) checks.push([{}]);
          checks[check_id  ][0] = first; checks[check_id  ][1] = second.front;
          checks[check_id+1][0] = first; checks[check_id+1][1] = second.back;
          check_id += 2;
        } else if (first instanceof BSP && second instanceof Jax.Geometry.Triangle) {
          // back is a tri, keep it to retest against front's children
          while (checks.length - check_id < 2) checks.push([{}]);
          checks[check_id  ][0] = first.front;  checks[check_id  ][1] = second;
          checks[check_id+1][0] = first.back;   checks[check_id+1][1] = second;
          check_id += 2;
        } else {
          // dealing with 2 triangles, perform intersection test
          // transform second into first's coordinate space
          mat4.multiplyVec3(transform, second.a, a);
          mat4.multiplyVec3(transform, second.b, b);
          mat4.multiplyVec3(transform, second.c, c);
          tri.set(a, b, c);
          
          if (first.intersectTriangle(tri)) {
            this.collision = {
              first: first,
              second: second,
              second_transformed: new Jax.Geometry.Triangle(tri.a, tri.b, tri.c)
            };
            return this.collision;
          }
        }
      }
      if (this.collision) delete this.collision;
      return false;
    },
    
    getCollision: function() { return this.collision; },
    
    getHalfSize: function() {
      return this.halfSize || this.calcHalfSize();
    },
    
    calcHalfSize: function() {
      this.halfSize = this.halfSize || vec3.create();
      var min, max;
      
      function calcSide(side) {
        var size, cmin, cmax;
        if (side instanceof BSP) size = side.getHalfSize();
        else size = calcTriangleDimensions(side);
        cmin = vec3.subtract(side.center, size, vec3.create());
        cmax = vec3.add(side.center, size, vec3.create());
        if (min) {
          vec3.min(min, cmin, min);
          vec3.max(max, cmax, max);
        } else {
          min = vec3.create(cmin);
          max = vec3.create(cmax);
        }
      }
      
      if (this.front) calcSide(this.front);
      if (this.back)  calcSide(this.back);
      
      vec3.subtract(max, min, this.halfSize);
      vec3.scale(this.halfSize, 0.5);

      return this.halfSize;
    },
    
    getCenter: function() {
      return this.center;
    },
    
    finalize: function() {
      var level = [];
      for (var i = 0; i < this.triangles.length; i++)
        level.push(this.triangles[i]);
      this.treeDepth = 1;
      while (level.length > 2) {
        buildLevel(level);
        this.treeDepth++;
      }
      this.set(level[0], level[1]);
      this.finalized = true;
    },
    
    getTreeDepth: function() { return this.treeDepth; },
    
    addTriangle: function(triangle) {
      this.triangles.push(triangle);
    },
    
    /**
     * BSP#traverse(point, callback) -> undefined
     * - point (vec3): the position of the camera, used for polygon sorting
     * - callback (Function): a callback function to call with each leaf
     *                              node as an argument
     * 
     * Traverses the BSP tree using the given point as a reference; leaf nodes
     * will be sent to the +callback+ function in back-to-front order.
     **/
    traverse: function(point, callback) {
      // handle the special case of having only 1 child
      if (!this.front || !this.back) {
        var result = this.front || this.back;
        if (result instanceof BSP) result.traverse(point, callback);
        else callback(result);
        return;
      }
      
      // find the distance from front to point, and from back to point
      var dist = vec3.create();
      vec3.subtract(this.front.center, point, dist);
      var frontLen = vec3.dot(dist, dist);
      vec3.subtract(this.back.center, point, dist);
      var backLen = vec3.dot(dist, dist);
      
      if (frontLen < backLen) {
        // closer to front, traverse back first
        if (this.back instanceof BSP) this.back.traverse(point, callback);
        else callback(this.back);
        
        if (this.front instanceof BSP) this.front.traverse(point, callback);
        else callback(this.front);
      } else {
        // closer to back, traverse front first
        if (this.front instanceof BSP) this.front.traverse(point, callback);
        else callback(this.front);

        if (this.back instanceof BSP) this.back.traverse(point, callback);
        else callback(this.back);
      }
    },
    
    addMesh: function(mesh) {
      var triangles = mesh.getTriangles();
      for (var i = 0; i < triangles.length; i++)
        this.addTriangle(triangles[i]);
    }
  });
})();