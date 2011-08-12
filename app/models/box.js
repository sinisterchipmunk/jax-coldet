var Box = (function() {
  var bufs = {};
  
  return Jax.Class.create({
    /**
     * new Box()
     * new Box(x, y, z, w, h, d)
     * new Box(pos, size)
     * new Box(box)
     * - x (Number): X position coordinate
     * - y (Number): Y position coordinate
     * - z (Number): Z position coordinate
     * - w (Number): box width
     * - h (Number): box height
     * - d (Number): box depth
     * - pos (vec3): position vector
     * - size (vec3): dimensions vector
     * - box (Box): box to clone
     *
     **/
    initialize: function() {
      this.position = vec3.create();
      this.size = vec3.create();
      this.center = vec3.create();
    
      switch(arguments.length) {
        case 0: break;
        case 6:
          this.position[0] = arguments[0];
          this.position[1] = arguments[1];
          this.position[2] = arguments[2];
          this.size[0] = arguments[3];
          this.size[1] = arguments[4];
          this.size[2] = arguments[5];
          break;
        case 2:
          vec3.set(arguments[0], this.position);
          vec3.set(arguments[1], this.size);
          break;
        case 1:
          vec3.set(arguments[0].getPosition(), this.position);
          vec3.set(arguments[0].getSize(), this.size);
          break;
        default: throw new Error("invalid arguments");
      }
    
      vec3.scale(this.size, 0.5, this.center);
      vec3.add(this.position, this.center, this.center);
    },
    
    toString: function() {
      return "[Box position:"+this.position+"; center:"+this.center+"; size:"+this.size+"]";
    },
  
    getPosition: function() { return this.position; },
    getSize: function() { return this.size; },
    getCenter: function() { return this.center; },
    getVolume: function() { return this.size[0] * this.size[1] * this.size[2]; },
  
    intersectRay: function(O, D, segmax) {
      if (segmax != undefined) return this.intersectLineSegment(O, D, segmax);

      var abs_segdir = vec3.create();
      var abs_cross = vec3.create();
      var f;
      var diff = vec3.create();
      var size = this.size;
      var cross = vec3.create();
    
      vec3.subtract(O, this.center, diff);

      for(i=0;i<3;i++)
      {
        abs_segdir[i] = Math.abs(D[i]);
        if (Math.abs(diff[i]) > size[i] && diff[i]*D[i] >= 0)
          return false;
      }

      vec3.cross(D, diff, cross);

      abs_cross[0] = Math.abs(cross[0]);
      f = size[1]*abs_segdir[2] + size[2]*abs_segdir[1];
      if (abs_cross[0] > f)
          return false;

      abs_cross[1] = Math.abs(cross[1]);
      f = size[0]*abs_segdir[2] + size[2]*abs_segdir[0];
      if (abs_cross[1] > f)
          return false;

      abs_cross[2] = Math.abs(cross[2]);
      f = size[0]*abs_segdir[1] + size[1]*abs_segdir[0];
      if (abs_cross[2] > f)
          return false;

      return true;
    },
  
    /**
     * Box#intersectLineSegment(O, D, segmax) -> Boolean
     * Box#intersectLineSegment(O, P) -> Boolean
     **/
    intersectLineSegment: function(O, D, segmax) {
      var tmp = vec3.create();
      
      if (segmax != undefined) {
        if (!isFinite(segmax)) return intersect(O,D); // infinite ray
        vec3.scale(D, segmax, tmp);
      } else {
        vec3.subtract(D, O, tmp);
        vec3.create();
        D = vec3.normalize(D, bufs.D);
      }
      
      var a0, a1, b0, b1;
      for (var i = 0; i < 3; i++) {
        a0 = this.position[i];
        a1 = this.position[i] + this.size[i];
        b0 = O[i];
        b1 = O[i] + tmp[i];
        var c;
        
        if (b0 < a0) { if (a0 >= b1) return false; }
        else           if (b0 >= a1) return false;
      }
      return true;
    },
  
    intersectSphere: function(O, radius) {
      var mx = vec3.create();
      vec3.add(this.position, this.size, mx);

      var dist = 0, d;
      for(var i=0;i<3;i++)
      {
        if (O[i] < this.position[i])
        {
          d = O[i] - this.position[i];
          dist += d*d;
        }
        else
        if (O[i] > mx[i])
        {
          d = O[i] - mx[i];
          dist += d*d;
        }
      }
      return (dist <= (radius*radius));
    },
  
    intersectPoint: function(p) {
      var pos = this.position;
      var s = this.size;
      if (p[0] < pos[0] || p[0] > (pos[0]+s[0])) return false;
      if (p[1] < pos[1] || p[1] > (pos[1]+s[1])) return false;
      if (p[2] < pos[2] || p[2] > (pos[2]+s[2])) return false;
      return true;
    },
  
    intersectAABB: function(b) {
      var t1 = this.position;
      var t2 = vec3.create();
      var p1 = b.getPosition();
      var p2 = vec3.create();
      vec3.add(t1, this.size, t2);
      vec3.add(p1, b.getSize(), p2);
    
      return (Math.max(p1[0], t1[0]) <= Math.min(p2[0], t2[0]) && 
              Math.max(p1[1], t1[1]) <= Math.min(p2[1], t2[1]) && 
              Math.max(p1[2], t1[2]) <= Math.min(p2[2], t2[2]));
    },
  
    intersectOBB: function(b, matrix) {
      var ra, rb;
      var a = this;
      var R = mat3.identity(mat3.create()), AbsR = mat3.identity(mat3.create());
      var i, j, tmp = vec3.create();
      
      // set up half-size extents
      a.e = vec3.scale(a.size, 0.5, vec3.create());
      b.e = vec3.scale(b.size, 0.5, vec3.create());
      
      // compute rotation matrix expressing b in a's coordinate frame
      a.u = [vec3.create(vec3.UNIT_X), vec3.create(vec3.UNIT_Y), vec3.create(vec3.UNIT_Z)];
      if (matrix) {
        var rm = mat3.create();
        // create normal matrix
        mat4.toInverseMat3(matrix, rm);
        mat3.transpose(rm);
        b.u = [
          mat3.multiplyVec3(rm, vec3.UNIT_X, vec3.create()),
          mat3.multiplyVec3(rm, vec3.UNIT_Y, vec3.create()),
          mat3.multiplyVec3(rm, vec3.UNIT_Z, vec3.create())
        ];
      }
      else
        b.u = [vec3.create(vec3.UNIT_X), vec3.create(vec3.UNIT_Y), vec3.create(vec3.UNIT_Z)];
      for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
          R[i*3+j] = vec3.dot(a.u[i], b.u[j]);
          
      // compute translation vector t
      var bc = mat4.multiplyVec3(matrix, vec3.create());
      vec3.add(bc, b.getCenter(), bc);
      var t = vec3.subtract(bc, a.getCenter(), vec3.create());
      // bring translation into a's coordinate frame
      tmp[0] = vec3.dot(t, a.u[0]);
      tmp[1] = vec3.dot(t, a.u[1]);
      tmp[2] = vec3.dot(t, a.u[2]);
      vec3.set(tmp, t);
      
      // Compute common subexpressions. Add in an epsilon term to
      // counteract arithmetic errors when two edges are parallel and
      // their cross product is near null.
      for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
          AbsR[i*3+j] = Math.abs(R[i*3+j]) + Math.EPSILON;
      
      // Test axes L = A0, L = A1, L = A2
      for (i = 0; i < 3; i++) {
        ra = a.e[i];
        rb = b.e[0] * AbsR[i*3+0] + b.e[1] * AbsR[i*3+1] + b.e[2] * AbsR[i*3+2];
        if (Math.abs(t[i]) > ra + rb)
          return false;
      }
      
      // Test axes L = B0, L = B1, L = B2
      for (i = 0; i < 3; i++) {
        ra = a.e[0] * AbsR[0*3+i] + a.e[1] * AbsR[1*3+i] + a.e[2] * AbsR[2*3+i];
        rb = b.e[i];
        if (Math.abs(t[0] * R[0*3+i] + t[1] * R[1*3+i] + t[2] * R[2*3+i]) > ra + rb)
          return false;
      }
      
      // Test axis L = A0 X B0
      ra = a.e[1] * AbsR[2*3+0] + a.e[2] * AbsR[1*3+0];
      rb = b.e[1] * AbsR[0*3+2] + b.e[2] * AbsR[0*3+1];
      if (Math.abs(t[2] * R[1*3+0] - t[1] * R[2*3+0]) > ra + rb)
        return false;
        
      // Test axis L = A0 X B1
      ra = a.e[1] * AbsR[2*3+2] + a.e[2] * AbsR[1*3+2];
      rb = b.e[0] * AbsR[0*3+1] + b.e[1] * AbsR[0*3+0];
      if (Math.abs(t[2] * R[1*3+1] - t[1] * R[2*3+1]) > ra + rb)
        return false;
      
      // Test axis L = A0 X B2
      ra = a.e[1] * AbsR[2*3+2] + a.e[2] * AbsR[1*3+2];
      rb = b.e[0] * AbsR[0*3+1] + b.e[1] * AbsR[0*3+0];
      if (Math.abs(t[2] * R[1*3+2] - t[1] * R[2*3+2]) > ra + rb)
        return false;

      // Test axis L = A1 X B0
      ra = a.e[0] * AbsR[2*3+0] + a.e[2] * AbsR[0*3+0];
      rb = b.e[1] * AbsR[1*3+2] + b.e[2] * AbsR[1*3+1];
      if (Math.abs(t[0] * R[2*3+0] - t[2] * R[0*3+0]) > ra + rb)
        return false;

      // Test axis L = A1 X B1
      ra = a.e[0] * AbsR[2*3+1] + a.e[2] * AbsR[0*3+1];
      rb = b.e[0] * AbsR[1*3+2] + b.e[2] * AbsR[1*3+0];
      if (Math.abs(t[0] * R[2*3+1] - t[2] * R[0*3+1]) > ra + rb)
        return false;

      // Test axis L = A1 X B2
      ra = a.e[0] * AbsR[2*3+2] + a.e[2] * AbsR[0*3+2];
      rb = b.e[0] * AbsR[1*3+1] + b.e[1] * AbsR[1*3+0];
      if (Math.abs(t[0] * R[2*3+2] - t[2] * R[0*3+2]) > ra + rb)
        return false;

      // Test axis L = A2 X B0
      ra = a.e[0] * AbsR[1*3+0] + a.e[1] * AbsR[0*3+0];
      rb = b.e[1] * AbsR[2*3+2] + b.e[2] * AbsR[2*3+1];
      if (Math.abs(t[1] * R[0*3+0] - t[0] * R[1*3+0]) > ra + rb)
        return false;

      // Test axis L = A2 X B1
      ra = a.e[0] * AbsR[1*3+1] + a.e[1] * AbsR[0*3+1];
      rb = b.e[0] * AbsR[2*3+2] + b.e[2] * AbsR[2*3+0];
      if (Math.abs(t[1] * R[0*3+1] - t[0] * R[1*3+1]) > ra + rb)
        return false;

      // Test axis L = A2 X B2
      ra = a.e[0] * AbsR[1*3+2] + a.e[1] * AbsR[0*3+2];
      rb = b.e[0] * AbsR[2*3+1] + b.e[1] * AbsR[2*3+0];
      if (Math.abs(t[1] * R[0*3+2] - t[0] * R[1*3+2]) > ra + rb)
        return false;

      // since no separating axes are found, the OBBs must be intersecting
      return true;
    }
  });
})();
