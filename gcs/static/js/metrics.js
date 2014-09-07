/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

var groups = [],

    // Define a new metric for display
    defineMetric = function (name, opts) {
        var m = {
                name: name,
                unit: "",
                type: "int",
                _range: {min: null, max: null}, // Global chart scale
                _target: null,
                _alert: null,
                _field: null,
                _interval: null,
                // d3 chart scale values
                xScale: null,
                yScale: null,

                // Array of scalar values for the chart
                valueData: [],
                // Array of scalar values for intervals
                intervalData: [],
                // Array of [min, max] tuples for the target area
                targetData: [],
                // Array of bools for alert status
                alertData: [],

                // Base this metric on some other already-defined metric,
                // inheriting any non-blank fields it may have.
                basedOn: (function (n) {
                        this.unit = n.unit || this.unit;
                        this.type = n.type || this.type;
                        this.range(n._range || this._range);
                        this._target = n._target || this._target;
                        this._alert = n._alert || this._alert;
                        this._field = n._field || this._field;
                        this._interval = n._interval || this._interval;

                        return this;
                    }),

                // Update this metric with new sample data
                update: (function (sample) {
                        var value = null, alert = false, target = null,
                            interval = null;

                        // Find the new sample value
                        if (!this._field) {
                            // No field defined, ignore
                            value = null;
                        } else if (typeof(this._field) === 'function') {
                            // Field is a function, so call it to generate the
                            // most recent value
                            value = this._field.call(this, sample);
                        } else {
                            // Field must be a key into the sample object
                            value = sample[this._field] !== undefined ?
                                sample[this._field] : null;
                        }

                        // Find the new interval value
                        if (!this._interval) {
                            // No interval defined, ignore
                            interval = null;
                        } else if (typeof(this._interval) === 'function') {
                            // Interval is a function, so call it to generate
                            // the most recent value
                            interval = this._interval.call(this, sample);
                        } else {
                            // Interval must be a key into the sample object
                            interval = sample[this._interval] !== undefined ?
                                       sample[this._interval] : null;
                        }

                        // Find the new target range
                        if (!this._target) {
                            // No target range, ignore
                            target = null;
                        } else if (typeof(this._target) === 'function') {
                            // Target is a function, pass it the latest sample
                            target = this._target.call(this, sample, value);
                        } else {
                            // Target is a {min: x, max: y} object
                            target = this._target;
                        }

                        // Find the new alert status value
                        if (!this._alert) {
                            // No alerts defined
                            alert = false;
                        } else if (typeof(this._alert) === 'function') {
                            // Alert is a function, so pass it the latest
                            // sample
                            alert = this._alert.call(this, sample, value);
                        } else {
                            // Alert must be a {min: x, max: y} object
                            if (this._alert.min !== undefined) {
                                alert = alert || (value < this._alert.min);
                            }
                            if (this._alert.max !== undefined) {
                                alert = alert || (value > this._alert.max);
                            }
                        }

                        this.valueData.push(value);
                        this.intervalData.push(interval);
                        this.targetData.push(target);
                        this.alertData.push(alert);

                        // Remove the first element of the arrays if they're
                        // more than 60 samples long
                        if (this.valueData.length == 62) {
                            this.valueData.shift();
                            this.intervalData.shift();
                            this.targetData.shift();
                            this.alertData.shift();
                        }

                        return this;
                    }),

                // Chainable getters/setters
                range: (function (min, max) {
                        if (min === undefined || min === null) {
                            return this._range;
                        } else if (typeof(min) === 'function') {
                            // Function returning a {min: A, max: B} object
                            this._range = min(this);
                        } else if (min.min !== undefined) {
                            // A {min: A, max: B} object has been passed
                            this._range = min;
                        } else {
                            // Two scalar values
                            this._range = {min: min, max: max};
                        }

                        if (this.type.split("(")[0] == "bool") {
                            this.xScale = d3.scale.linear().domain([61, 0]);
                            this.yScale = d3.scale.linear().domain([-1, 1]);
                        } else {
                            this.xScale = d3.scale.linear().domain([61, 0]);
                            this.yScale = d3.scale.linear()
                                            .domain([this._range.min,
                                                     this._range.max]);
                        }

                        return this;
                    }),
                target: (function (min, max) {
                        if (min === undefined || min === null) {
                            return this._target;
                        } else if (typeof(min) === 'function') {
                            this._target = min;
                        } else {
                            this._target = {min: min, max: max};
                        }
                        return this;
                    }),
                alert: (function (min, max) {
                        if (min === undefined || min === null) {
                            return this._alert;
                        } else if (typeof(min) === 'function') {
                            this._alert = min;
                        } else {
                            this._alert = {min: min, max: max};
                        }
                        return this;
                    }),
                field: (function (f) {
                        if (f === undefined || f === null) {
                            return this._field;
                        } else {
                            this._field = f;
                        }
                        return this;
                    }),
                interval: (function (i) {
                        if (i === undefined || i === null) {
                            return this._interval;
                        } else {
                            this._interval = i;
                        }
                        return this;
                    })
            };
        if (opts) {
            m.unit = opts.unit || m.unit;
            m.type = opts.type || m.type;
            m.range(opts.range || m._range);
            m._target = opts.target || m._target;
            m._alert = opts.alert || m._alert;
            m._field = opts.field || m._field;
            m._interval = opts.interval || m._interval;
        } else {
            m.range(0, 100);
        }
        return m;
    },

    // Define a new metric group for display in the dashboard
    defineGroup = function (name) {
        var g = {
            name: name,
            metrics: [],

            // Remove this group from the global groups list
            destroy: (function () {
                    var idx = groups.indexOf(this);
                    if (idx !== -1) {
                        groups.splice(idx, 1);
                    }
                    return this;
                }),

            // Add a new metric to the group
            add: (function (m) {
                    this.metrics.push(m);
                    return this;
                }),

            // Remove a metric from the group
            remove: (function (m) {
                    var idx = this.metrics.indexOf(m);
                    if (idx !== -1) {
                        this.metrics.splice(idx, 1);
                    }
                    return this;
                }),

            // Update metrics based on a new sample
            update: (function (sample) {
                    for (var i = 0, l = this.metrics.length; i < l; i++) {
                        this.metrics[i].update(sample);
                    }
                })
        };
        groups.push(g);
        return g;
    };

var tmpl = {
        distance: defineMetric("distance", {unit: "m", type: "float(1)"}),
        speed: defineMetric("speed",
            {unit: "m/s", type: "float(1)", range: {min: 0, max: 40}}),
        velocity: defineMetric("velocity",
            {unit: "m/s", type: "float(1)", range: {min: -40, max: 40}}),
        angle: defineMetric("angle",
            {unit: "°", type: "float(0)", range: {min: -180, max: 180}}),
        heading: defineMetric("heading",
            {unit: "°", type: "float(0)", range: {min: 0, max: 360}}),
        signal: defineMetric("signal",
            {unit: "dBm", type: "float(0)", range: {min: -128, max: 0}}),
        signalMargin: defineMetric("signalMargin",
            {unit: "dBm", type: "float(0)", range: {min: 0, max: 128}}),
        count: defineMetric("count", {unit: "", type: "int"}),
        fault: defineMetric("fault",
            {unit: "", type: "bool(,fault)",
            alert: function(samp, val) { return !val; }}),
        percentage: defineMetric("percentage",
            {unit: "%", type: "int", range: {min: 0, max: 100}}),
        charge: defineMetric("charge", {unit: "A*h", type: "float(1)"}),
        text: defineMetric("text", {unit: "", type: "text"})
    };
