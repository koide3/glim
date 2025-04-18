# Manual Object Removal

## Video

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/FSkNsVNoCU4?si=MbCYOm-z9gbB_bbd" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

## Example

**Example dump data**:  

  - [inarimae_mmm.tar.gz](https://staff.aist.go.jp/k.koide/projects/glim/datasets/inarimae_mmm.tar.gz) [245MB] (3 sessions in large outdoor)

### Start map editor

```bash
ros2 run glim_ros map_editor
```
### Open a mapping result

- `File` -> `Open New Map` -> Select a dump directory and click `Open`.

### Object selection and removal (MinCut segmentation)

- Right click a point on an object to be removed to open a context menu.
- `Segmentation` -> Select `MinCut` -> Adjust `Foreground radius` and `Background radius`
    - Points in the foreground radius (shown by a red sphere) are labeled as foreground
    - Points outside the background radius (shown by a blue sphere) are labeled as background
- Click `Segment` to segment the object
- Click `Remove selected points` to remove the selected points from the map

| Context menu  | Selected points | After removal |
|---|---|---|
|![Image](https://github.com/user-attachments/assets/c7322108-f704-46ce-94e4-1b8c16a8b1a5)|![Image](https://github.com/user-attachments/assets/d1fb6f53-dd23-4dbe-a47a-6778f037221d)|![Image](https://github.com/user-attachments/assets/31597ecc-ec93-44ed-8a55-9c62d74dff5d)|


### Plane selection and removal (Region growing segmentation)

- Right click a point on a plane to be removed to open a context menu.
- `Segmentation` -> Select `RegionGrowing` -> Adjust `Angle threshold` and `Distance threshold`
- Click `Segment` to segment the plane
- Click `Remove selected points` to remove the selected points from the map

| Context menu  | Selected points |
|---|---|
|![Image](https://github.com/user-attachments/assets/3b7b0d86-e771-422e-b77d-578ff7ecc7e9)|![Image](https://github.com/user-attachments/assets/fc6815fb-de56-479e-9fa0-b1d6dc2b1c7a)|


### Manual points selection and removal (Gizmo UI)

- Click the checkbox just left to `Selection tool` to show the Gizmo UI.
    - It is also possible to show the Gizmo UI by clicking `Show Gizmo here` from the right-click context menu.
- Manipulate the Gizmo UI to cover the points to be removed.
- Click `Select points` to select the points covered by the Gizmo UI.
- Click `Remove selected points` to remove the selected points from the map

| Gizmo UI  | Selected points |
|---|---|
|![Image](https://github.com/user-attachments/assets/8754d4c4-856d-43fd-95f6-3443af5ce142)|![Image](https://github.com/user-attachments/assets/cefabf73-986e-4bc4-aba7-3e8bb53a4cf9)|

### Radius selection and removal

1. Right click a point on the map to open a context menu.
2. `Radius tools` -> Adjust `Radius` so that it covers the points to be removed -> Click `Select points within radius`.
3. Click `Remove selected points` to remove the selected points from the map


### Radius outlier removal (Noise removal)

1. Right click a point on the map to open a context menu.
2. `Radius tools` -> Adjust `Radius` so that it covers the outlier points to be removed -> Click `Select points outside radius`.
3. Click `Remove selected points` to remove the selected points from the map


| Context menu  | Selected points | After removal |
|---|---|---|
|![Image](https://github.com/user-attachments/assets/8c8561e7-7ecc-4f8f-ba16-2642dcf6899d)|![Image](https://github.com/user-attachments/assets/73c729b3-111a-469b-a02f-2623d8269534)|![Image](https://github.com/user-attachments/assets/e394e75f-09bb-4a9e-b509-90d1789069eb)|



