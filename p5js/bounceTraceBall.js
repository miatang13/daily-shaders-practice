// 02/10/21
// Mia Tang
// https://editor.p5js.org/miatang13/sketches/Rav_2nRy0

var posTopL;
var posTopR;
var posBotL;
var posBotR;
var positions;
var circ;
var e;
var traces = [];
const TRACE_RADIUS = 15;
const TRACE_NUM = 15;
const CIRCLE_RADIUS = 30;
const BD = CIRCLE_RADIUS * 2; // padding to the boundary
const SHUFFLE_TIME = 2000; // milliseconds
const A = 0.15; // for damped sinusoid 

function setup() {
  createCanvas(400, 400);
  ellipseMode(CENTER);
  posTopL = new createVector(BD, height - BD);
  posTopR = new createVector(width - BD, height - BD);
  posBotL = new createVector(BD, BD);
  posBotR = new createVector(width - BD, BD);
  positions = [posTopL, posTopR, posBotR, posBotL]
  e = new p5.Ease(); // easing function object
  circ = new glideCircle();
}

function draw() {
  background(0);
  let posX = (circ.getPos()).x;
  let posY = (circ.getPos()).y;

  if (traces.length > TRACE_NUM) {
    traces.shift();
  }
  traces.push(new Trace(posX + TRACE_RADIUS, posY + TRACE_RADIUS, TRACE_RADIUS));
  traces.forEach(trace => trace.draw());
  circ.draw();
}

class glideCircle {
  constructor() {
    this.posIdx = 0;
    this.timeAtPos = millis();;
  }

  getPos() {
    let curProg = millis() - this.timeAtPos;
    if (curProg > SHUFFLE_TIME) {
      this.posIdx = (this.posIdx + 1) % positions.length;
      this.timeAtPos = millis();
      curProg = 0;
    }
    let curPosIdxPos = positions[this.posIdx];
    let newPos = positions[(this.posIdx + 1) % positions.length]
    let posPercent = map(curProg, 0, SHUFFLE_TIME, 0, 1)
    let posPercentTweak = e.elasticInOut(posPercent);
    let posX = map(posPercentTweak, 0, 1, curPosIdxPos.x, newPos.x);
    let posY = map(posPercentTweak, 0, 1, curPosIdxPos.y, newPos.y)
    return new createVector(posX, posY);
  }

  draw() {
    push();
    fill("white");
    noStroke();
    ellipse((this.getPos()).x, (this.getPos()).y, CIRCLE_RADIUS);
    pop();
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
      ellipse(this.x - this.r, this.y - this.r, this.x + this.r, this.y + this.r)
    }
  }
}
