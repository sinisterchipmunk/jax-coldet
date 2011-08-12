describe("BSP", function() {
  var bsp;
  beforeEach(function() { bsp = new BSP(); });
  
  describe("two separate BSP spheres", function() {
    var mesh, bsp1, bsp2, cam1, cam2;
    
    beforeEach(function() {
      mesh = new Jax.Mesh.Sphere();
      bsp1 = new BSP();
      bsp2 = new BSP();
      bsp1.addMesh(mesh);
      bsp2.addMesh(mesh);
      cam1 = new Jax.Camera();
      cam2 = new Jax.Camera();
    });
    
    function mat() {
      var mat = mat4.create();
      mat4.inverse(cam1.getTransformationMatrix(), mat);
      mat4.multiply(mat, cam2.getTransformationMatrix(), mat);
      return mat;
    }
    
    it("should collide", function() {
      expect(bsp1.collide(bsp2, mat())).toBeTruthy();
    });
    
    it("should not collide if out of range", function() {
      cam2.setPosition([0,2.1,0]);
      expect(bsp1.collide(bsp2, mat())).toBeFalsy();
    });
    
    it("should not collide if cubes intersect, but meshes do not", function() {
      // as spheres have radius 1, we'll move 1.1 units diagonally; because of the diagonal,
      // cubes will still intersect, but spheres will not.
      cam2.setPosition(vec3.scale(vec3.normalize([1,1,0]), 2.3));
      expect(bsp1.collide(bsp2, mat())).toBeFalsy();
    });
  });
  
  describe("with a single triangle", function() {
    beforeEach(function() {
      bsp.addTriangle(new Jax.Geometry.Triangle([0,1,0],  [-1,0,0],  [1,0,0]));
      bsp.finalize();
    });
    
    it("should be only 1 node deep", function() {
      expect(bsp.front).toBeKindOf(Jax.Geometry.Triangle);
      expect(bsp.back).toBeFalsy();
    });
    
    it("should have accurate halfSize", function() {
      expect(bsp.getHalfSize()).toEqualVector([1,0.5,Math.EPSILON]);
    });
  });
  
  describe("with 2 triangles", function() {
    beforeEach(function() {
      bsp.addTriangle(new Jax.Geometry.Triangle([0,1,0],  [-1,0,0],  [1, 0,0]));
      bsp.addTriangle(new Jax.Geometry.Triangle([1,0,0],  [-1,0,0],  [0,-1,0]));
      bsp.finalize();
    });
    
    it("should have accurate halfSize", function() {
      expect(bsp.getHalfSize()).toEqualVector([1,0.8333333730697632,Math.EPSILON]);
    });
    
    it("should be only 1 node deep", function() {
      expect(bsp.front).toBeKindOf(Jax.Geometry.Triangle);
      expect(bsp.back).toBeKindOf(Jax.Geometry.Triangle);
    });
  });
  
  describe("with 3 triangles", function() {
    beforeEach(function() {
      bsp.addTriangle(new Jax.Geometry.Triangle([2,1,0],  [ 1,0,0],  [3,0,0]));
      bsp.addTriangle(new Jax.Geometry.Triangle([0,1,0],  [-1,0,0],  [1,0,0]));
      bsp.addTriangle(new Jax.Geometry.Triangle([1,0,0],  [-1,0,0],  [0,-1,0]));
      bsp.finalize();
    });
    
    it("should be 2 nodes deep", function() {
      expect(bsp.front).toBeKindOf(BSP);
      expect(bsp.back).toBeKindOf(Jax.Geometry.Triangle);
      expect(bsp.front.front).toBeKindOf(Jax.Geometry.Triangle);
      expect(bsp.front.back).toBeKindOf(Jax.Geometry.Triangle);
    });
  });

  describe("with 4 triangles", function() {
    beforeEach(function() {
      bsp.addTriangle(new Jax.Geometry.Triangle([2,1,0],  [ 1,0,0],  [3,0,0]));
      bsp.addTriangle(new Jax.Geometry.Triangle([0,1,0],  [-1,0,0],  [1,0,0]));
      bsp.addTriangle(new Jax.Geometry.Triangle([1,0,0],  [-1,0,0],  [0,-1,0]));
      bsp.addTriangle(new Jax.Geometry.Triangle([3,0,0],  [ 1,0,0],  [2,-1,0]));
      bsp.finalize();
    });
    
    it("should be 2 nodes deep", function() {
      expect(bsp.getTreeDepth()).toEqual(2);
      
      expect(bsp.front).toBeKindOf(BSP);
      expect(bsp.back).toBeKindOf(BSP);
      expect(bsp.front.front).toBeKindOf(Jax.Geometry.Triangle);
      expect(bsp.front.back).toBeKindOf(Jax.Geometry.Triangle);
      expect(bsp.back.front).toBeKindOf(Jax.Geometry.Triangle);
      expect(bsp.back.back).toBeKindOf(Jax.Geometry.Triangle);
    });
  });
});
