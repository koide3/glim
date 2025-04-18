# Manual Object Removal

## About

Interactive map editor allows users to manually annotate and remove object points from the map.
This is useful for removing objects that are not relevant to the mapping task, such as trees, cars, and pedestrians.

Note that while editing the map, the submap poses (i.e., factor graph) are frozen and not updated. If you need to update the submap poses, please use the offline viewer.

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
|[![Image](https://github.com/user-attachments/assets/96352a2d-7d86-46c5-9dd8-916366fd0867)](https://github.com/user-attachments/assets/96352a2d-7d86-46c5-9dd8-916366fd0867)|[![Image](https://github.com/user-attachments/assets/648ec8ae-fc4d-45ff-8209-81da6837b188)](https://github.com/user-attachments/assets/648ec8ae-fc4d-45ff-8209-81da6837b188)|[![Image](https://github.com/user-attachments/assets/3b722598-a622-49a0-872d-5a714a291d85)](https://github.com/user-attachments/assets/3b722598-a622-49a0-872d-5a714a291d85)|


### Plane selection and removal (Region growing segmentation)

- Right click a point on a plane to be removed to open a context menu.
- `Segmentation` -> Select `RegionGrowing` -> Adjust `Angle threshold` and `Distance threshold`
- Click `Segment` to segment the plane
- Click `Remove selected points` to remove the selected points from the map

| Context menu  | Selected points |
|---|---|
| [![Image](https://github.com/user-attachments/assets/3e4ebcaf-f9bc-40dd-8b03-b2f7a138e5cf)](https://github.com/user-attachments/assets/3e4ebcaf-f9bc-40dd-8b03-b2f7a138e5cf) | [![Image](https://github.com/user-attachments/assets/f12cd4ca-78ef-48f9-a1ab-c33d712b4545)](https://github.com/user-attachments/assets/f12cd4ca-78ef-48f9-a1ab-c33d712b4545) |



### Manual points selection and removal (Gizmo UI)

- Click the checkbox just left to `Selection tool` to show the Gizmo UI.
    - It is also possible to show the Gizmo UI by clicking `Show Gizmo here` from the right-click context menu.
- Manipulate the Gizmo UI to cover the points to be removed.
- Click `Select points` to select the points covered by the Gizmo UI.
- Click `Remove selected points` to remove the selected points from the map

| Gizmo UI  | Selected points |
|---|---|
| [![Image](https://github.com/user-attachments/assets/3906c3cd-0db9-4fbc-9aa1-3a2ab81b8ce6)](https://github.com/user-attachments/assets/3906c3cd-0db9-4fbc-9aa1-3a2ab81b8ce6) | [![Image](https://github.com/user-attachments/assets/7e55d910-2f9e-4991-9ae5-7c5c5e28a6a0)](https://github.com/user-attachments/assets/7e55d910-2f9e-4991-9ae5-7c5c5e28a6a0) |

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
| [![Image](https://github.com/user-attachments/assets/856b4113-9f17-40db-9eb7-3b9e08fb6408)](https://github.com/user-attachments/assets/856b4113-9f17-40db-9eb7-3b9e08fb6408) | [![Image](https://github.com/user-attachments/assets/89375d06-5f41-4ea2-bc0d-71e591be91bb)](https://github.com/user-attachments/assets/89375d06-5f41-4ea2-bc0d-71e591be91bb) | [![Image](https://github.com/user-attachments/assets/814b846b-4225-4653-b519-1501a653761c)](https://github.com/user-attachments/assets/814b846b-4225-4653-b519-1501a653761c) |


