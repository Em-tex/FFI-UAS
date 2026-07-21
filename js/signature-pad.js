/* js/signature-pad.js */

function SignaturePad(canvas, options) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.onChange = (options && options.onChange) || function () {};
    this.empty = true;
    this._setupCanvas();
    this._bindEvents();
}

SignaturePad.prototype._setupCanvas = function () {
    const ratio = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, Math.round(rect.width * ratio));
    this.canvas.height = Math.max(1, Math.round(rect.height * ratio));
    this.ctx.scale(ratio, ratio);
    this.ctx.lineWidth = 2.2;
    this.ctx.lineCap = "round";
    this.ctx.lineJoin = "round";
    this.ctx.strokeStyle = "#1a1a1a";
};

SignaturePad.prototype._bindEvents = function () {
    const self = this;
    let drawing = false;
    let lastX = 0;
    let lastY = 0;

    this.canvas.addEventListener("pointerdown", function (e) {
        drawing = true;
        self.empty = false;
        lastX = e.offsetX;
        lastY = e.offsetY;
        self.canvas.setPointerCapture(e.pointerId);
        e.preventDefault();
    });

    this.canvas.addEventListener("pointermove", function (e) {
        if (!drawing) return;
        self.ctx.beginPath();
        self.ctx.moveTo(lastX, lastY);
        self.ctx.lineTo(e.offsetX, e.offsetY);
        self.ctx.stroke();
        lastX = e.offsetX;
        lastY = e.offsetY;
        e.preventDefault();
    });

    function stop() {
        if (!drawing) return;
        drawing = false;
        self.onChange();
    }

    this.canvas.addEventListener("pointerup", stop);
    this.canvas.addEventListener("pointercancel", stop);
    this.canvas.addEventListener("pointerleave", stop);
};

SignaturePad.prototype.clear = function () {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.empty = true;
    this.onChange();
};

SignaturePad.prototype.isEmpty = function () {
    return this.empty;
};

SignaturePad.prototype.toDataURL = function () {
    return this.empty ? "" : this.canvas.toDataURL("image/png");
};

SignaturePad.prototype.fromDataURL = function (dataUrl) {
    if (!dataUrl) return;
    const self = this;
    const ratio = window.devicePixelRatio || 1;
    const img = new Image();
    img.onload = function () {
        self.ctx.drawImage(img, 0, 0, self.canvas.width / ratio, self.canvas.height / ratio);
        self.empty = false;
    };
    img.src = dataUrl;
};

function initSignaturePads(ids, onChange) {
    const pads = {};
    ids.forEach(function (id) {
        const canvas = document.getElementById(id);
        if (!canvas) return;
        pads[id] = new SignaturePad(canvas, { onChange: onChange });
    });
    document.querySelectorAll(".signature-clear-btn[data-clear-target]").forEach(function (btn) {
        const targetId = btn.getAttribute("data-clear-target");
        btn.addEventListener("click", function () {
            if (pads[targetId]) pads[targetId].clear();
        });
    });
    return pads;
}
