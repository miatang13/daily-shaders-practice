// 02/13/21
// Mia Tang

var posTopL;
var posTopR;
var posBotL;
var posBotR;
var positions;
var circ;
var e;
var circles = [];
const CIRCLE_NUM = 15
const CIRCLE_RADIUS = 30;
const TRACE_RADIUS = CIRCLE_RADIUS * 3;
const TRACE_NUM = 15;
const BD = CIRCLE_RADIUS * 3; // padding to the boundary
const SHUFFLE_TIME = 2000; // milliseconds
const A = 0.15; // for damped sinusoid 

function setup() {
  createCanvas(400, 400);
  ellipseMode(CENTER);
  angleMode(DEGREES);
  setUpPositions();
  e = new p5.Ease(); // easing function object
  for (var i = 0; i < CIRCLE_NUM; i++) {
    circles.push(new glideCircle(i % positions.length, positions));
  }
}

function setUpPositions() {
  positions = [];
  let angle = 0;
  let r = width / 3;
  let angleIncre = 360 / CIRCLE_NUM;
  for (var i = 0; i < CIRCLE_NUM; i++) {
    let n = createVector(r * cos(angle), r * sin(angle))
    positions.push(n);
    angle += angleIncre;
  }
}

function draw() {
  translate(width / 2, height / 2)
  background(0);
  push();
  circles.forEach(circ => {
    circ.update();
    circ.draw();
  });
  pop();
}

class glideCircle {
  constructor(posIdx, positions) {
    this.posIdx = posIdx;
    this.x = (positions[posIdx]).x;
    this.y = (positions[posIdx]).y;
    this.timeAtPos = millis();
    this.radius = CIRCLE_RADIUS;
    this.traces = [];
  }

  update() {
    this.updatePosition();
    this.updateTrace();
  }

  updatePosition() {
    let curProg = millis() - this.timeAtPos;
    if (curProg > SHUFFLE_TIME) {
      this.posIdx = (this.posIdx + 5) % positions.length;
      this.timeAtPos = millis();
      curProg = 0;
    }
    let newPos = positions[(this.posIdx + 8) % positions.length]
    let posPercent = map(curProg, 0, SHUFFLE_TIME, 0, 1)
    //  let posPercentTweak = e.elasticInOut(posPercent);
    // let posPercentTweak = e.doubleSquircularSigmoid(posPercent, A/2, 0.5);
    // let posPercentTweak = e.doubleSquircularOgee(posPercent, A/1.5, 0.5);
    
   // let posPercentTweak = e.exponentialEmphasis(posPercent, A);
    let posPercentTweak = e.catmullRomInterpolate (posPercent, 5.0, .5);
    let posX = map(posPercentTweak, 0, 1, this.x, newPos.x);
    let posY = map(posPercentTweak, 0, 1, this.y, newPos.y)
    this.x = posX;
    this.y = posY;
  }

  updateTrace() {
    if (this.traces.length > TRACE_NUM) {
      this.traces.shift();
    }
    this.traces.push(new Trace(this.x, this.y, TRACE_RADIUS));
  }

  draw() {
    push();
    // fill("white");
    noFill();
    stroke('white')
    //noStroke();
    ellipse(this.x, this.y, this.radius);
    pop();
    this.traces.forEach(trace => trace.draw());
  }
}

class Trace {
  constructor(x, y, r) {
    this.x = x;
    this.y = y;
    this.r = r;
    this.birthday = millis();
  }

  draw() {
    let now = millis();
    let hasBeen = now - this.birthday;
    let hasBeenPer = hasBeen / 3000;

    if ((hasBeenPer > 0) && (hasBeenPer < 1)) {
      var alpha = map(hasBeenPer, 0, 1, 155, 0);
      var r = map(hasBeenPer, 0, 1, 20, 1)
      push();
      noFill();
      stroke(alpha);
      //ellipse(this.x - this.r, this.y - this.r, this.x + this.r, this.y + this.r)
      ellipse(this.x, this.y, this.r);
    }
  }
}
