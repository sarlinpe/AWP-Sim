//
// "Configuration Space Visualization of 2-D Robotic Manipulator"
//
// Copyright (C), 2010 Jeffrey Ichnowski
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//
//   * The names of its contributors may be used to endorse or promote
//     products derived from this software without specific prior
//     written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

window.addEventListener("load", function() {

    // Configuration
    var DIMS = 400;

    // ======================================================================
    // Utility methods
    // ======================================================================

    function set(obj, kvpairs) {
        for (var key in kvpairs) {
            if (kvpairs.hasOwnProperty(key)) {
                if ("object" == typeof obj[key]) {
                    set(obj[key], kvpairs[key]);
                } else {
                    obj[key] = kvpairs[key];
                }
            }
        }
        return obj;
    }


    // Returns the (x,y) position of an event from an SVG object.
   function eventPos(evt) {
        if ("undefined" != typeof evt.offsetX) {
            // WebKit
            return {x:evt.offsetX, y:evt.offsetY};
        } else {
            // Firefox
            var sse = evt.target.ownerSVGElement;
            var ctm = sse.getScreenCTM();
            var pt = sse.createSVGPoint();
            pt.x = evt.clientX;
            pt.y = evt.clientY;
            var pt2 = pt.matrixTransform(ctm.inverse());
            return pt2;
        }
    }

    // Returns the (x,y) position of the event relative to the target
    // of the event
    function mousePosInTarget(evt) {
        var x = evt.clientX + (0|document.documentElement.scrollLeft) + (0|document.body.scrollLeft);
        var y = evt.clientY + (0|document.documentElement.scrollTop) + (0|document.body.scrollTop);
        var elem = evt.target;
        do {
            x -= elem.offsetLeft;
            y -= elem.offsetTop;
        } while (elem = elem.offsetParent);
        return {x:x,y:y};
    }

    function setAttrs(e, attrs) {
        for (var key in attrs) {
            if (attrs.hasOwnProperty(key)) {
                e.setAttribute(key, String(attrs[key]));
            }
        }
        return e;
    }

    function createSvg(type, attrs) {
        return setAttrs(document.createElementNS("http://www.w3.org/2000/svg", type), attrs);
    }

    function forceRedraw(svgArea) {
        svgArea.ownerSVGElement.forceRedraw();
    }

    function dist2(x1,y1, x2,y2) {
        var dx = x2 - x1;
        var dy = y2 - y1;
        return dx*dx + dy*dy;
    }

    // Normalizes an angle to the range 0..2*Math.PI.
    function normalizeAngle(a) {
        if (a < 0) return 2*Math.PI + a % (2*Math.PI);
        if (a < 2*Math.PI) return a;
        return a % (2*Math.PI);
    }

    // returns the angle between two angles to the range 0..Math.PI
    function angleBetween(a, b) {
        return Math.min(normalizeAngle(a-b), normalizeAngle(b-a));
    }

    function segmentIntersects(x1, y1, x2, y2, x3,y3 , x4,y4) {
        // early out with bounds check
        var xMin12, xMax12, xMin34, xMax34;
        if (x1 < x2) { xMin12=x1; xMax12=x2; } else { xMin12=x2; xMax12=x1; }
        if (x3 < x4) { xMin34=x3; xMax34=x4; } else { xMin34=x4; xMax34=x3; }
        if (xMax12 < xMin34 || xMax34 < xMin12) {
            return null;
        }
        var yMin12, yMax12, yMin34, yMax34;
        if (y1 < y2) { yMin12=y1; yMax12=y2; } else { yMin12=y2; yMax12=y1; }
        if (y3 < y4) { yMin34=y3; yMax34=y4; } else { yMin34=y4; yMax34=y3; }
        if (yMax12 < yMin34 || yMax34 < yMin12) {
            return null;
        }

        var x1_x2 = x1-x2;
        var y1_y2 = y1-y2;
        var x3_x4 = x3-x4;
        var y3_y4 = y3-y4;

        var d = x1_x2*y3_y4 - y1_y2*x3_x4;

        if (d === 0) {
            // lines are parallel
            return null;
        }

        var x1y2_y1x2 = (x1*y2 - y1*x2);
        var x3y4_y3x4 = (x3*y4 - y3*x4);

        var x = (x1y2_y1x2*x3_x4 - x1_x2*x3y4_y3x4) / d;
        if (x+0.001 < Math.max(Math.min(x1,x2), Math.min(x3,x4)) ||
            x-0.001 > Math.min(Math.max(x1,x2), Math.max(x3,x4)))
        {
            return null;
        }
        var y = (x1y2_y1x2*y3_y4 - y1_y2*x3y4_y3x4) / d;
        if (y+0.001 < Math.max(Math.min(y1,y2), Math.min(y3,y4)) ||
            y-0.001 > Math.min(Math.max(y1,y2), Math.max(y3,y4)))
        {
            return null;
        }

        return {x:x, y:y};
    }

    var DEG2RAD = Math.PI / 180;

    function $(id) {
        return document.getElementById(id);
    }

    // In theory we could support IE's attachEvent with this simple
    // abstraction.
    function addEventListener(dom,evt,handler) {
        dom.addEventListener(evt, handler, false);
    }
    
    // ======================================================================
    // Matrix Class (TBD: just use SVGMatrix)
    // ======================================================================

    // a c e
    // b d f
    // 0 0 1
    var Matrix = set(function(a,b,c,d,e,f) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
        this.f = f;
        // console.log("new matrix: "+this);
    },{
        prototype : {
            multiply : function(that) {
                return new Matrix(
                    this.a * that.a + this.c * that.b, // a
                    this.b * that.a + this.d * that.b, // b
                    this.a * that.c + this.c * that.d, // c
                    this.b * that.c + this.d * that.d, // d
                    this.a * that.e + this.c * that.f + this.e, // e
                    this.b * that.e + this.d * that.f + this.f); // f
            },
            transform : function(pos) {
                return { x : this.a * pos.x + this.c * pos.y + this.e,
                         y : this.b * pos.x + this.d * pos.y + this.f };
            },
            invert : function() {
                var z = 1.0 / (this.a * this.d - this.c * this.b);
                return new Matrix(
                    this.d * z, // a
                   -this.b * z, // b
                   -this.c * z, // c
                    this.a * z, // d
                    (this.c * this.f - this.e * this.d) * z, // e
                    (this.e * this.b - this.a * this.f) * z); // f
            },
            toString : function() {
                return "[ "+this.a+" "+this.c+" "+this.e+" / "+this.b+" "+this.d+" "+this.f+" / 0 0 1 ]";
            }
        },
        translate : function(x, y) { return new Matrix(1,0,0,1,x,y); },
        scale : function(x, y) { return new Matrix(x,0,0,y,0,0); },
        rotate : function(a) {
            a = a * DEG2RAD;
            var s = Math.sin(a);
            var c = Math.cos(a);
            return new Matrix(c,s,-s,c,0,0);
        }
    });
    Matrix.IDENTITY = new Matrix(1,0,0,1,0,0);

    // ======================================================================
    // A Simple Drag and Drop Manager
    // ======================================================================

    var DnDManager = (function() {
        var dragMatrix = null;
        var dragCallback = null;
        var endCallback = null;
        var dragStart;

        // Capture starting coordinates of drag
        addEventListener(window, "mousedown", function(evt) {
            if (dragCallback) {
                dragStart = dragMatrix.transform(eventPos(evt));
                // console.log("dragStart: "+dragStart.x+", "+dragStart.y);
            }
        });

        // Update coordinates during drag
        addEventListener(window, "mousemove", function(evt) {
            if (dragCallback) {
                var pos = dragMatrix.transform(eventPos(evt));
                dragCallback(pos.x - dragStart.x, pos.y - dragStart.y);
                dragStart = pos;
            }
        });
        
        // End the drag
        addEventListener(window, "mouseup", function(evt) {
            if (dragCallback) {
                dragCallback = null;
                if (endCallback) {
                    endCallback();
                }
            }
        });

        return {
            mode: null,
            startDrag:function (callback, matrix, ec) {
                matrix = matrix.invert();
                // console.log("startDrag: "+matrix);
                dragCallback = callback;
                dragMatrix = matrix;
                endCallback = ec;
            },
            isDragging:function() {
                return !!dragCallback;
            }
        };
    })();

    // ======================================================================
    // User Interface Queries
    // ======================================================================

    function getRobotMode() {
        var options = [ "setup", "config", "inverse" ];
        for (var i=options.length ; --i >= 0 ; ) {
            var o = options[i];
            if ($("robotMode-"+o).checked) {
                return o;
            }
        }
        return options[0];
    }

    // ======================================================================
    // User Interface Setup
    // ======================================================================
    $("robotMode-setup").checked = true;

    DnDManager.mode = getRobotMode();

    // ======================================================================
    // The Robot
    // ======================================================================
    var RobotPart = set(function() {}, {
        prototype : {
            attach : function(comp) {
                this.next = comp;
                comp.prev = this;
            },
            getLocalTransform :function() {
                return Matrix.IDENTITY;
            },
            getTransform : function() {
                var mine = this.getLocalTransform();
                if (this.prev) {
                    return this.prev.getTransform().multiply(mine);
                } else {
                    return mine;
                }
            },
            getParentTransform : function() {
                return this.prev ? this.prev.getTransform() : Matrix.IDENTITY;
            }
        }
    });

    function Link(len, ang, name) {
        this.length = len;
        this.angle = ang;
        this.name = name;

        var self = this;

        addEventListener($(name+"end"), "mousedown", function() {
            DnDManager.mode = getRobotMode();
            if (DnDManager.mode == "inverse") {
                return;
            }
            DnDManager.startDrag(
                function(dx, dy) {
                    var oldPos = self.getPosition();
                    var x = oldPos.x + dx;
                    var y = oldPos.y + dy;
                    
                    if (DnDManager.mode == "setup") {
                        self.setLength(Math.sqrt(x*x + y*y));
                    }

                    self.setAngle(Math.atan2(y, x) / DEG2RAD);
                },
                self.getParentTransform(),
                DnDManager.mode == "setup" ? cSpace.update : null);
        });
    }
    Link.prototype = set(new RobotPart(), {
        getLocalTransform : function() {
            return Matrix.translate(this.length, 0).multiply(
                Matrix.rotate(this.angle));
        },
        getPosition : function() {
            var angle = this.angle * DEG2RAD;
            return { x : Math.cos(angle) * this.length,
                     y : Math.sin(angle) * this.length };
        },
        setPosition : function(x, y) {
            this.setLength(Math.sqrt(x*x + y*y));
            this.setAngle(Math.atan2(y, x) / DEG2RAD);
        },
        setLength : function(len) {
            this.length = len;
            $(this.name+"scale").setAttribute("transform", "scale("+len+" 1)");
            $(this.name+"trans").setAttribute("transform", "translate("+len+")");
        },
        setAngle : function(angle) {
            this.angle = angle;
            $(this.name+"rot").setAttribute("transform", "rotate("+angle+")");
            cSpace.moveX();
            workspace.updateLaser();
        }
    });

    function Robot() {
        this.x = 200;
        this.y = 200;
        this.name = "robot";

        var robotElem = $(this.name);
        var self = this;

        // Make the base movable
        for (var i=1 ; i<=2 ; ++i) {
            addEventListener($(this.name+"Base"+i),
                "mousedown", function(evt) {
                    DnDManager.mode = getRobotMode();
                    if (DnDManager.mode == "setup") {
                        DnDManager.startDrag(function(dx, dy) {
                            self.setPosition(self.x + dx, self.y + dy);
                        }, self.getParentTransform(), cSpace.update);
                    }
                });
        }

        var link = new Link(75, 45, "arm1");
        this.attach(link);
        link.attach(new Link(50, 90, "arm2"));
    }
    Robot.prototype = set(new RobotPart(), {
        setPosition : function(x, y) {
            this.x = x;
            this.y = y;
            var elem = $(this.name);
            elem.setAttribute("transform", "translate("+x+" "+y+")");
        },
        getLocalTransform : function() {
            return Matrix.translate(this.x, this.y);
        }
    });

    var robot = new Robot();

    // ======================================================================
    // The Obstacles
    // ======================================================================

    var obstacles = [];
    var workspace = (function() {
        function Polygon(parent, color) {
            this.parent = parent;
            this.color = color;
            this.rgb = "rgb("+color+")";
            this.points = [];
            this.bounds = { x1:Infinity, y1:Infinity, x2:-Infinity, y2:-Infinity };
        }
        Polygon.prototype = {
            isClosing : function(x,y) {
                // console.log("isClosing: "+x+","+y);
                var n = this.points.length;
                // close the polygon if clicking within 4 pixels of
                // the first or last point (e.g. closing the polygon
                // directly, or double-clicking on the last point)
                if (n > 2 && (dist2(x,y, this.points[0][0], this.points[0][1]) <= 16 ||
                              dist2(x,y, this.points[n-1][0], this.points[n-1][1]) <= 16)) {
                    return true;
                }
                return false;
            },
            add : function(x,y) {
                this.points.push([x,y]);
                this.bounds.x1 = Math.min(this.bounds.x1, x);
                this.bounds.y1 = Math.min(this.bounds.y1, y);
                this.bounds.x2 = Math.max(this.bounds.x2, x);
                this.bounds.y2 = Math.max(this.bounds.y2, y);
                switch (this.points.length) {
                case 1:
                    // Put up a single dot
                    this.elem = this.parent.appendChild(
                        createSvg("circle", {r:1, fill:this.rgb, cx:x, cy:y}));
                    forceRedraw(this.parent);
                    break;
                case 2:
                    // 2 points: put up a line segement
                    this.parent.removeChild(this.elem);
                    this.elem = this.parent.appendChild(
                        createSvg("line", {
                            stroke:this.rgb, "stroke-width":1,
                            x1:this.points[0][0], y1:this.points[0][1],
                            x2:this.points[1][0], y2:this.points[1][1]}));
                    forceRedraw(this.parent);
                    break;
                case 3:
                    // 3 segements: start a polyline
                    this.parent.removeChild(this.elem);
                    this.elem = this.parent.appendChild(
                        createSvg("polyline", {
                            fill:"none", stroke:this.rgb, points:this.points.join(" ")}));
                    forceRedraw(this.parent);
                    break;
                default:
                    // 4 segements or more: add to the polyline
                    setAttrs(this.elem, { points:this.points.join(" ") });
                    // this.elem.setAttribute("points", this.points.join(" "));
                    forceRedraw(this.parent);
                    break;
                }
            },
            close : function() {
                if (this.points.length > 2) {
                    this.parent.removeChild(this.elem);
                    this.elem = this.parent.appendChild(
                        createSvg("polygon", {
                            fill:this.rgb, stroke:"black", points:this.points.join(" ") }));
                    var self = this;
                    addEventListener(this.elem, "mousedown", function(evt) {
                        if (getRobotMode() == "setup") {
                            DnDManager.startDrag(function(dx,dy) {
                                self.bounds.x1 += dx;
                                self.bounds.y1 += dy;
                                self.bounds.x2 += dx;
                                self.bounds.y2 += dy;
                                for (var i=0 ; i<self.points.length ; ++i) {
                                    self.points[i][0] += dx;
                                    self.points[i][1] += dy;
                                }
                                setAttrs(self.elem, {points:self.points.join(" ")});
                                // self.elem.setAttribute("points", self.points.join(" "));
                            }, Matrix.IDENTITY, cSpace.update);
                        }
                    });
                }
            },
            remove : function() {
                this.parent.removeChild(this.elem);
            },
            intersectsSegment : function(x1, y1, x2, y2) {
                var b = this.bounds;
                if (x1 < b.x1 && x2 < b.x1 ||
                    x1 > b.x2 && x2 > b.x2 ||
                    y1 < b.y1 && y2 < b.y1 ||
                    y1 > b.y2 && y2 > b.y2)
                {
                    // not within rectangular bounds
                    return false;
                }
                
                var pts = this.points;
                var i = pts.length - 1;
                
                if (segmentIntersects(x1, y1, x2, y2, pts[0][0], pts[0][1], pts[i][0], pts[i][1])) {
                    return true;
                }
                
                while (--i >= 0) {
                    if (segmentIntersects(x1, y1, x2, y2, pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])) {
                        return true;
                    }
                }
                
                return false;
            }
        };
        
        // See: http://www.w3.org/TR/SVG/types.html#ColorKeywords
        var colors = [
            [255,0,0], // red
            [255,165,0], // orange
            [0, 128, 0], // green
            [0, 0, 255], // blue
            [128, 0, 128], // purple
            [128, 128, 128] // gray
        ];
        
        var activePolygon = null;
        var robotPaint = $("robotPaint");
        var obstaclesGroup = $("obstacles");
        var lastClickPos = {x:-1,y:-100};
        
        function obstaclesIntersectSegment(x1,y1,x2,y2) {
            for (var i=0 ; i<obstacles.length ; ++i) {
                if (obstacles[i].intersectsSegment(x1, y1, x2, y2)) {
                    return true;
                }
            }
            return false;
        }
        
        function isValidConfiguration(angle1, angle2) {
            var x1 = robot.x;
            var y1 = robot.y;
            var x2 = x1 + Math.cos(angle1) * robot.next.length;
            var y2 = y1 + Math.sin(angle1) * robot.next.length;
            
            if (obstaclesIntersectSegment(x1, y1, x2, y2)) {
                return false;
            }
            
            var x3 = x2 + Math.cos(angle1 + angle2) * robot.next.next.length;
            var y3 = y2 + Math.sin(angle1 + angle2) * robot.next.next.length;
            
            return !obstaclesIntersectSegment(x2, y2, x3, y3);
        }

        // The "Laser" are the beams that connect the points on the robot.
        // This method makes the beams turn red or green based upon
        // collision situation
        function updateLaser() {
            var stops = $("myGradient").getElementsByTagName("stop");
            var color = isValidConfiguration(robot.next.angle * DEG2RAD,
                                             robot.next.next.angle * DEG2RAD) ?
                "#0f0" : "#f00";
            [0,1,4,5].forEach(function(x) {
                stops[x].setAttribute("stop-color", color);
            });
            //         stops[0].setAttribute("stop-color", color);
            //         stops[1].setAttribute("stop-color", color);
            //         stops[4].setAttribute("stop-color", color);
            //         stops[5].setAttribute("stop-color", color);
        }
        
        updateLaser();

        addEventListener($("removeAll"), "click", function(evt) {
            for (var i=0 ; i<obstacles.length ; ++i) {
                obstacles[i].remove();
            }
            obstacles.length = 0;
            cSpace.update();
            $("removeAll").disabled = true;
            return false;
        });
        
        // Event handler for stopping any active setup operations
        function leaveSetupMode(evt) {
            if (evt.target.checked && activePolygon) {
                activePolygon.remove();
                activePolygon = null;
                obstaclesGroup.removeChild(paintBand);
            }
        }
        
        addEventListener($("robotMode-config"), "click", leaveSetupMode);
        addEventListener($("robotMode-inverse"), "click", leaveSetupMode);
        
        var paintBand = null;
        var mouseIsDown = false;
        
        function inverseKinematics(evt) {
            var pos = eventPos(evt);
            var px = pos.x - robot.x;
            var py = pos.y - robot.y;
            var len1 = robot.next.length;
            var len2 = robot.next.next.length;
            var m = (len1*len1 + len2*len2 - px*px - py*py) / (2 * len1 * len2);
            
            var theta1, theta2;
            
            function computeTheta1(theta2) {
                return Math.atan2(py, px) -
                    Math.atan2(len2*Math.sin(theta2), len1 + len2*Math.cos(theta2));
            }
            
            if (m < -1) {
                // not reachable, arm fully extended
                theta2 = 0;
                theta1 = Math.atan2(py, px);
            } else if (m > 1) {
                // not reachable, arm completely retracted
                theta2 = Math.PI;
                theta1 = (py*px) ? Math.atan2(py, px) : 0;
            } else {
                theta2 = Math.PI-Math.acos(m);
                // For fun: if (Math.random() < .5) theta2 *= -1;
                
                var oldTheta2 = robot.next.next.angle * DEG2RAD;
                
                // We can choose between theta2 and -theta2 As a tie
                // breaker we choose the one closest to the previous
                // value of theta2
                if (angleBetween(oldTheta2, -theta2) < angleBetween(oldTheta2, theta2)) {
                    theta2 = -theta2;
                }
                
                theta1 = computeTheta1(theta2);
                
                if (!isValidConfiguration(theta1, theta2)) {
                    // if theta2 is invalid, try -theta2
                    
                    var altTheta1 = computeTheta1(-theta2);
                    if (isValidConfiguration(altTheta1, -theta2)) {
                        theta2 = -theta2;
                        theta1 = altTheta1;
                    }
                }
            }
            
            // console.log("m="+m+", theta1 = "+theta1+", theta2="+theta2);
            
            robot.next.setAngle(theta1 / DEG2RAD);
            robot.next.next.setAngle(theta2 / DEG2RAD);
        }
        
        addEventListener(robotPaint, "mousedown", function(evt) {
            if (getRobotMode() == "inverse") {
                mouseIsDown = true;
                inverseKinematics(evt);
            }
        });
        addEventListener(window, "mouseup", function(evt) { mouseIsDown = false; });
        addEventListener(robotPaint, "click", function(evt) {
            // console.log("click on paint");
            if (activePolygon) {
                var pos = eventPos(evt);
                if (activePolygon.isClosing(pos.x, pos.y)) {
                    activePolygon.close();
                    obstacles.push(activePolygon);
                    activePolygon = null;
                    cSpace.update();
                    $("removeAll").disabled = false;
                    obstaclesGroup.removeChild(paintBand);
                } else {
                    activePolygon.add(pos.x, pos.y);
                    obstaclesGroup.appendChild(paintBand); // move to last
                    setAttrs(paintBand, {x1:pos.x, y1:pos.y});
                    // paintBand.setAttribute("x1", String(pos.x));
                    // paintBand.setAttribute("y1", String(pos.y));
                }
            }
        });
        addEventListener(robotPaint, "mousemove", function(evt) {
            if (activePolygon) {
                var pos = eventPos(evt);
                setAttrs(paintBand, {x2:String(pos.x), y2:String(pos.y)});
                //paintBand.setAttribute("x2", String(pos.x));
                //paintBand.setAttribute("y2", String(pos.y));
            }
            if (getRobotMode() == "inverse" && mouseIsDown) {
                inverseKinematics(evt);
            }
        });
        addEventListener($("robotBG"), "click", function(evt) {
            if (!DnDManager.isDragging() && getRobotMode() == "setup") {
                var pos = eventPos(evt);
                if (!activePolygon) {
                    activePolygon = new Polygon(
                        obstaclesGroup,
                        colors[obstacles.length % colors.length]);
                    paintBand = createSvg("line", {stroke:"black",
                                                   "stroke-width":1,
                                                   "stroke-dasharray":"4,2",
                                                   "stroke-dashoffset":2,
                                                   x1:pos.x, y1:pos.y,
                                                   x2:pos.x, y2:pos.y});
                    var paintAnim = createSvg("animate", {
                        attributeName:"stroke-dashoffset",
                        attributeType:"XML",
                        from:"0",
                        to:"6",
                        dur:"6s",
                        repeatCount:"indefinite"
                    });
                    paintBand.appendChild(paintAnim);
                    obstaclesGroup.appendChild(paintBand);
                }
                lastClickPos = pos;
            }
        });
        
        return { isValidConfiguration : isValidConfiguration, updateLaser : updateLaser };
    })();

    // ======================================================================
    // Create a "cSpace" (configuration space) canvas object
    // ======================================================================

    var cSpace = (function() {
        var updateIndex = -1;
        var canvas = $("canvas");
        var ctx = canvas.getContext("2d");

        var imgData, pixels;

        var cSpaceMoving = false;
        function cSpaceMove(evt) {
            var pos = mousePosInTarget(evt);
            // console.log("canvas: "+pos.x+", "+pos.y);
            robot.next.setAngle(pos.x * 360 / DIMS);
            robot.next.next.setAngle(pos.y * 360 / DIMS);
        }

        addEventListener(canvas, "mousedown", function(evt) {
            cSpaceMove(evt);
            cSpaceMoving = true;
        });

        addEventListener(canvas, "mousemove", function(evt) {
            if (cSpaceMoving) {
                cSpaceMove(evt);
            }
        });

        addEventListener(window, "mouseup", function(evt) {
            cSpaceMoving = false;
        });


        function newImage() {
            imgData = ctx.createImageData(DIMS, DIMS);
            pixels = imgData.data;
        }

        newImage();

        function updateCanvas() {
            ctx.clearRect(0, 0, DIMS, DIMS);

            if (obstacles.length === 0) {
                drawX();
                return;
            }

            function renderUpdatingText() {
                var updatingText = "Updating C-Space...";
                ctx.fillText(updatingText,
                             (DIMS - ctx.measureText(updatingText).width) / 2,
                             (DIMS - 10) / 2);
            }

            newImage();

            // ctx.putImageData(imgData, 0, 0);

            renderUpdatingText();

            updateIndex = 0;
            setTimeout(iteration, 1);

            function iteration() {
                if (updateIndex >= obstacles.length) {
                    ctx.putImageData(imgData, 0, 0);
                }

                var oi = updateIndex;
                renderObstacle(obstacles[oi]);

                ctx.putImageData(imgData, 0, 0);
                drawX();

                if (++updateIndex < obstacles.length) {
                    renderUpdatingText();
                    setTimeout(iteration, 1);
                }
            }
            
            function renderObstacle(obstacle) {
                var color = obstacle.color;
                function setPixel() {
                    var a = pixels[pixOffset+3];
                    if (a) {
                        pixels[pixOffset  ] = (r + pixels[pixOffset  ])>>1;
                        pixels[pixOffset+1] = (g + pixels[pixOffset+1])>>1;
                        pixels[pixOffset+2] = (b + pixels[pixOffset+2])>>1;
                    } else {
                        pixels[pixOffset  ] = r;
                        pixels[pixOffset+1] = g;
                        pixels[pixOffset+2] = b;
                        pixels[pixOffset+3] = 0xff;
                    }
                }
                var r = color[0];
                var g = color[1];
                var b = color[2];

                var x1 = robot.x;
                var y1 = robot.y;

                for (var ai=0 ; ai<DIMS ; ++ai) {
                    var aa = ai * Math.PI * 2 / DIMS;

                    var x2 = x1 + Math.cos(aa) * robot.next.length;
                    var y2 = y1 + Math.sin(aa) * robot.next.length;

                    var pixOffset = ai * 4;
                    if (obstacle.intersectsSegment(x1, y1, x2, y2)) {
                        for (var bi=0 ; bi<DIMS ; ++bi) {
                            setPixel();
                            pixOffset += DIMS * 4;
                        }
                    } else {
                        var len2 = robot.next.next.length;
                        
                        for (var bi=0 ; bi<DIMS ; ++bi) {
                            var ba = (ai+bi) * Math.PI * 2 / DIMS;
                            var x3 = x2 + Math.cos(ba) * len2;
                            var y3 = y2 + Math.sin(ba) * len2;
                            if (obstacle.intersectsSegment(x2, y2, x3, y3)) {
                                setPixel();
                            }
                            pixOffset += DIMS * 4;
                        }
                    }
                }
            }
        }

        function drawX() {
            var x = robot.next.angle * DIMS / 360;
            var y = robot.next.next.angle * DIMS / 360;
            x = x + DIMS*(1+(-x / DIMS)|0);
            y = y + DIMS*(1+(-y / DIMS)|0);
            ctx.fillRect(x-3, y, 7, 1);
            ctx.fillRect(x, y-3, 1, 7);
        }

        return {
            update: function() {
                updateCanvas();
                workspace.updateLaser();
            },
            moveX : function() {
                if (obstacles.length) {
                    ctx.putImageData(imgData, 0, 0);
                } else {
                    ctx.clearRect(0, 0, DIMS, DIMS);
                }
                drawX();
            }
        };
    })();
}, false);


