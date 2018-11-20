
class Rectangle {
  float x;
  float y;
  float z;
  float w;
  float h;
  float d;
  
  Rectangle(float x, float y, float z, float w, float h, float d) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
    this.h = h;
    this.d = d;
  }

  Boolean contains(PVector point) {
    return (point.x >= this.x - this.w &&
      point.x <= this.x + this.w &&
      point.y >= this.y - this.h &&
      point.y <= this.y + this.h &&
      point.z >= this.z - this.d &&
      point.z <= this.z + this.d);
  }


  Boolean intersects(Rectangle range) {
    return !(range.x - range.w > this.x + this.w ||
      range.x + range.w < this.x - this.w ||
      range.y - range.h > this.y + this.h ||
      range.y + range.h < this.y - this.h ||
      range.z - range.d > this.z + this.d ||
      range.z + range.d < this.z - this.d);
  }


}

class Circle {
  
  PVector center;
  float r;
  
  Circle(PVector c, float r) {
    this.center = c;
    this.r = r;
  }

  Boolean contains(PVector p) {
    // check if the point is in the circle by checking if the euclidean distance of
    // the point and the center of the circle if smaller or equal to the radius of
    // the circle
    Boolean d = false;
    if (p.dist(this.center) <= r)
      d = true;
    
    return d;
  }

/*  Boolean intersects(PVector c; Rectangle range) {

    float xDist = abs(c.x - this.center.x);
    float yDist = abs(c.y - this.center.y);
    float zDist = abs(c.z - this.center.z);

    // radius of the circle
    float r = this.r;

    var w = range.w;
    var h = range.h;

    var edges = Math.pow((xDist - w), 2) + Math.pow((yDist - h), 2);

    // no intersection
    if (xDist > (r + w) || yDist > (r + h))
      return false;

    // intersection within the circle
    if (xDist <= w || yDist <= h)
      return true;

    // intersection on the edge of the circle
    return edges <= this.rSquared;
  }*/
}

class octtree {
  
  Rectangle boundary;
  int capacity;
  ArrayList<PVector> points = new ArrayList<PVector>();
  Boolean divided;
    
  octtree Fnortheast;
  octtree Fnorthwest;
  octtree Fsoutheast;
  octtree Fsouthwest;
      
  octtree Bnortheast;
  octtree Bnorthwest;
  octtree Bsoutheast;
  octtree Bsouthwest;
    
  octtree(Rectangle b, int c) {
    this.boundary = b;
    this.capacity = c;
    this.divided = false;
    

  }
  

  void subdivide() {
    float x = this.boundary.x;
    float y = this.boundary.y;
    float z = this.boundary.z;
    float w = this.boundary.w / 2;
    float h = this.boundary.h / 2;
    float d = this.boundary.d / 2;

    Rectangle Fne = new Rectangle(x + w, y - h, z - d, w, h, d);
    this.Fnortheast = new octtree(Fne, this.capacity);
    Rectangle Fnw = new Rectangle(x - w, y - h, z - d, w, h, d);
    this.Fnorthwest = new octtree(Fnw, this.capacity);
    Rectangle Fse = new Rectangle(x + w, y + h, z - d, w, h, d);
    this.Fsoutheast = new octtree(Fse, this.capacity);
    Rectangle Fsw = new Rectangle(x - w, y + h, z - d, w, h, d);
    this.Fsouthwest = new octtree(Fsw, this.capacity);

    Rectangle Bne = new Rectangle(x + w, y - h, z + d, w, h, d);
    this.Bnortheast = new octtree(Bne, this.capacity);
    Rectangle Bnw = new Rectangle(x - w, y - h, z + d, w, h, d);
    this.Bnorthwest = new octtree(Bnw, this.capacity);
    Rectangle Bse = new Rectangle(x + w, y + h, z + d, w, h, d);
    this.Bsoutheast = new octtree(Bse, this.capacity);
    Rectangle Bsw = new Rectangle(x - w, y + h, z + d, w, h, d);
    this.Bsouthwest = new octtree(Bsw, this.capacity);

    this.divided = true;
  }

  Boolean insert(PVector point) {
    if (!this.boundary.contains(point)) {
      return false;
    }

    if (this.points.size() < this.capacity) {
      this.points.add(point);
      return true;
    }

    if (!this.divided) {
      subdivide();
    }

    if (this.Fnortheast.insert(point) || this.Fnorthwest.insert(point) ||
      this.Fsoutheast.insert(point) || this.Fsouthwest.insert(point) ||
      this.Bnortheast.insert(point) || this.Bnorthwest.insert(point) ||
      this.Bsoutheast.insert(point) || this.Bsouthwest.insert(point)) {
      return true;
    } else 
    return false;
  }

  ArrayList<PVector> query(Rectangle range) {
   
    
    ArrayList<PVector> found = new ArrayList<PVector>();
  
    if (!range.intersects(this.boundary)) {
      return found;
    }

    for (int i = 0; i < this.points.size(); i++) {
      if (range.contains(this.points.get(i))) {
        found.add(this.points.get(i));
      }
    }
    
    if (this.divided) {
     found.addAll(this.Fnorthwest.query(range));
     found.addAll(this.Fnortheast.query(range));
     found.addAll(this.Fsouthwest.query(range));
     found.addAll(this.Fsoutheast.query(range));
     found.addAll(this.Bnorthwest.query(range));
     found.addAll(this.Bnortheast.query(range));
     found.addAll(this.Bsouthwest.query(range));
     found.addAll(this.Bsoutheast.query(range));
    }

    
    return found;
  }

}
