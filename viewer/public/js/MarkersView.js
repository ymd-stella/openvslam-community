class MarkersView {

    constructor() {

        /* Camera componet */
        this.MARKER_MATERIAL = new THREE.LineBasicMaterial({ color: "rgb(0, 255, 0)" });

        this.markerIndices = [];
        this.markerObjects = [];
        this.markerCorners = [];
        this.addedMarkerIndices = [];
        this.removedPool = [];
        this.removedPoolSize = 0;

        this.totalMarkerCount = 0;
        this.numValidMarker = 0;

        this.POOL_CORNERS = [[0., 0., 0.], [1., 0., 0.], [1., 0., 1.], [0., 0., 1.]];
    }

    // private methods

    addMarker(id, corners) {

        if (this.removedPoolSize > 0) {
            let index = this.removedPool.pop();
            this.removedPoolSize--;

            this.markerIndices[id] = index;
            this.changeMarkerPos(index, pose);
        }
        else {
            let lineGeometry = this.makeWireframe(corners);
            let markerObject = new THREE.LineSegments(lineGeometry, this.MARKER_MATERIAL);

            this.markerIndices[id] = this.totalMarkerCount;
            this.addedMarkerIndices.push(this.totalMarkerCount);
            this.markerObjects[this.totalMarkerCount] = markerObject;
            this.markerCorners[this.totalMarkerCount] = corners;

            this.totalMarkerCount++;
        }

        this.numValidMarker++;
    }

    remove(id) {

        let index = this.markerIndices[id];

        if (this.markerIndices[id] < 0 || index === undefined)
            return;

        this.changeMarkerPos(index, this.POOL_CORNERS);

        this.markerIndices[id] = -1;
        this.removedPool.push(index);
        this.removedPoolSize++;

        this.numValidMarker--;
    }

    changeMarkerPos(index, corners) {
        let lineGeometry = this.makeWireframe(corners);

        this.markerObjects[index].geometry = lineGeometry;
        this.markerObjects[index].geometry.verticesNeedUpdate = true;
        this.markerCorners[index] = corners;
    }

    makeWireframe(corners) {
        let lineGeo = new THREE.Geometry();
        // center to 4 vertices
        lineGeo.vertices.push(
            new THREE.Vector3(corners[0].coords[0] * GLOBAL_SCALE, corners[0].coords[1] * GLOBAL_SCALE, corners[0].coords[2] * GLOBAL_SCALE),
            new THREE.Vector3(corners[1].coords[0] * GLOBAL_SCALE, corners[1].coords[1] * GLOBAL_SCALE, corners[1].coords[2] * GLOBAL_SCALE),
            new THREE.Vector3(corners[1].coords[0] * GLOBAL_SCALE, corners[1].coords[1] * GLOBAL_SCALE, corners[1].coords[2] * GLOBAL_SCALE),
            new THREE.Vector3(corners[2].coords[0] * GLOBAL_SCALE, corners[2].coords[1] * GLOBAL_SCALE, corners[2].coords[2] * GLOBAL_SCALE),
            new THREE.Vector3(corners[2].coords[0] * GLOBAL_SCALE, corners[2].coords[1] * GLOBAL_SCALE, corners[2].coords[2] * GLOBAL_SCALE),
            new THREE.Vector3(corners[3].coords[0] * GLOBAL_SCALE, corners[3].coords[1] * GLOBAL_SCALE, corners[3].coords[2] * GLOBAL_SCALE),
            new THREE.Vector3(corners[3].coords[0] * GLOBAL_SCALE, corners[3].coords[1] * GLOBAL_SCALE, corners[3].coords[2] * GLOBAL_SCALE),
            new THREE.Vector3(corners[0].coords[0] * GLOBAL_SCALE, corners[0].coords[1] * GLOBAL_SCALE, corners[0].coords[2] * GLOBAL_SCALE),);
        return lineGeo;
    }

    // public methods

    update(id, corners) {
        let index = this.markerIndices[id];

        if (index < 0 || index === undefined) {
            this.addMarker(id, corners);
        }
        else {
            this.changeMarkerPos(index, corners);
        }
    }

    updateMarkersInScene(scene) {
        for (let index in this.addedMarkerIndices) {
            scene.add(this.markerObjects[index]);
        }
    }

}
