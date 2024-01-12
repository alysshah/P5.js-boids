// Code built off of https://p5js.org/examples/simulate-flocking.html
// Additional inspiration: Dan Shiffman's Flocking Simulation

let flock;
let confinementMode = 0;

//////////////////////// FLOCK CLASS ////////////////////////
class Flock {
  constructor() {
    this.boids = [];
  }

  run() {
    for (let i = 0; i < this.boids.length; i++) {
      // Pass the entire list of boids to each boid individually
      this.boids[i].run(this.boids);
    }
  }

  addBoid(b) {
    this.boids.push(b);
  }
}

//////////////////////// BOID CLASS ////////////////////////
class Boid {
  constructor(x, y) {
    this.acceleration = createVector(0, 0);
    this.velocity = createVector(random(-1, 1), random(-1, 1));
    this.position = createVector(x, y);
    this.r = 2.0; // Boid size
    this.maxspeed = 3; // Max speed
    this.maxforce = 0.2; // Max steering force
  }

  run(boids) {
    this.flock(boids);
    this.update();
    this.borders();
    this.render();
  }

  applyForce(force) {
    // We could add mass here if we want A = F / M
    this.acceleration.add(force);
  }

  flock(boids) {
    let sep = this.separation(boids); // Separation
    let ali = this.alignment(boids); // Alignment
    let coh = this.cohesion(boids); // Cohesion
    let avo = this.avoid(); // Mouse repulsion

    // Force weight
    sep.mult(1.5);
    ali.mult(1.0);
    coh.mult(1.0);
    avo.mult(2.5);

    // Add the force vectors to acceleration
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
    this.applyForce(avo);
  }

  // Update position
  update() {
    // Update velocity
    this.velocity.add(this.acceleration);
    // Limit speed
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    // Reset accelertion to 0 each cycle
    this.acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  seek(target) {
    let desired = p5.Vector.sub(target, this.position); // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(this.maxspeed);
    // Steering = Desired minus Velocity
    let steer = p5.Vector.sub(desired, this.velocity);
    steer.limit(this.maxforce); // Limit to maximum steering force
    return steer;
  }

  // Render method for each boid
  render() {
    let speed = this.velocity.mag(); // Calculate current speed
    // Draw the boid with the calculated color
    let theta = this.velocity.heading() + radians(90);
    let col = getColorFromSpeed(speed, this.maxspeed); // Get color based on speed
    fill(col);
    stroke(col);
    push();
    translate(this.position.x, this.position.y);
    rotate(theta);
    beginShape();
    vertex(0, -this.r * 2);
    vertex(-this.r, this.r * 2);
    vertex(this.r, this.r * 2);
    endShape(CLOSE);
    pop();
  }

  borders() {
    if (confinementMode === 0) {
      // Original wraparound logic
      if (this.position.x < -this.r) this.position.x = width + this.r;
      if (this.position.y < -this.r) this.position.y = height + this.r;
      if (this.position.x > width + this.r) this.position.x = -this.r;
      if (this.position.y > height + this.r) this.position.y = -this.r;
    } else if (confinementMode === 1) {
      // STILL WORKING ON THIS
      // Rectangular walls
      this.position.x = constrain(this.position.x, 0, width);
      this.position.y = constrain(this.position.y, 0, height);
    } // WILL ADD OPTION FOR CIRCULAR WALLS
  }

  // Separation
  separation(boids) {
    let desiredseparation = 25.0;
    let steer = createVector(0, 0);
    let count = 0;
    // For every boid in the system, check if it's too close
    for (const other of boids) {
      let d = p5.Vector.dist(this.position, other.position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if (d > 0 && d < desiredseparation) {
        // Calculate vector pointing away from neighbor
        let diff = p5.Vector.sub(this.position, other.position);
        diff.normalize();
        diff.div(d); // Weight by distance
        steer.add(diff);
        count++; // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div(count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(this.maxspeed);
      steer.sub(this.velocity);
      steer.limit(this.maxforce);
    }
    return steer;
  }

  // Alignment
  alignment(boids) {
    let neighbordist = 50;
    let sum = createVector(0, 0);
    let count = 0;
    for (const other of boids) {
      let d = p5.Vector.dist(this.position, other.position);
      if (d > 0 && d < neighbordist) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      sum.normalize();
      sum.mult(this.maxspeed);
      let steer = p5.Vector.sub(sum, this.velocity);
      steer.limit(this.maxforce);
      return steer;
    } else {
      return createVector(0, 0);
    }
  }

  // Cohesion
  cohesion(boids) {
    let neighbordist = 50;
    let sum = createVector(0, 0); // Start with empty vector to accumulate all locations
    let count = 0;
    for (const other of boids) {
      let d = p5.Vector.dist(this.position, other.position);
      if (d > 0 && d < neighbordist) {
        sum.add(other.position); // Add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return this.seek(sum); // Steer towards the location
    } else {
      return createVector(0, 0);
    }
  }

  // Avoid (for mouse repulsion)
  avoid() {
    let mouse = createVector(mouseX, mouseY);
    let desiredseparation = 100.0; // Change this value to affect the repulsion distance
    let steer = createVector(0, 0);
    let d = p5.Vector.dist(this.position, mouse);

    if (d > 0 && d < desiredseparation) {
      let diff = p5.Vector.sub(this.position, mouse);
      diff.normalize();
      diff.div(d); // Weight by distance
      steer.add(diff);
    }

    if (steer.mag() > 0) {
      steer.normalize();
      steer.mult(this.maxspeed);
      steer.sub(this.velocity);
      steer.limit(this.maxforce);
    }

    return steer;
  }
}

//////////////////////// HELPER FUNCTIONS ////////////////////////

// Function to map boid speed to color
function getColorFromSpeed(speed, maxSpeed) {
  let percent = speed / maxSpeed; // Normalize speed to a 0-1 range
  let endColor = color(0, 0, 255); // Blue (fast)
  let startColor = color(255, 0, 50); // Red (slow)
  let lerpedColor = lerpColor(startColor, endColor, percent); // Linear interpolation between the 2 colors
  return lerpedColor;
}

//////////////////////// SKETCH ////////////////////////

function setup() {
  createCanvas(windowWidth, windowHeight);
  flock = new Flock();
  for (let i = 0; i < 300; i++) {
    flock.addBoid(new Boid(random(0, width), random(0, height)));
  }
}

function draw() {
  background(0);
  flock.run();
}
