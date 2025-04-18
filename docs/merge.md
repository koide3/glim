# Merging Sessions

## About

You can merge multiple mapping sessions into a single map using the offline viewer. This is useful when you want to create a large map from multiple mapping sessions, or when you want to improve the global consistency of the map by merging multiple sessions.

!!!note
    The map data is likely to be broken when warnings like `[global] [warning] X34 -> E69 is missing` are shown, and the optimization may get corrupted when merging broken map data. Before merging sessions, make sure each map data is not broken. A broken map can be repaired by clicking `Recover graph`. You have to repair maps one by one (Load a broken map -> repair it -> save it -> close it, then do so for the next map).

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/aMq3qbAgTeI?si=QRZqp0DjSK79NcQk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</div>

## Example

**Example dump data**:  

  - [mmm.tar.gz [32MB]](https://staff.aist.go.jp/k.koide/projects/glim/datasets/mmm.tar.gz) (3 indoor sessions)
  - [inarimae_mmm.tar.gz [245MB]](https://staff.aist.go.jp/k.koide/projects/glim/datasets/inarimae_mmm.tar.gz) (3 sessions in large outdoor)
  
### Start offline viewer

```bash
ros2 run glim_ros offline_viewer
```

### Open mapping sessions

1. Open the first session : `File` -> `Open New Map` -> `Select the first dump directory` -> Select `Yes` on the dialog to enable optimization.
2. Open the second session : `File` -> `Open Additional Map` -> `Select the second dump directory`.

[![Image](https://github.com/user-attachments/assets/ad96720e-8dee-4e81-adbd-ac370bd0c646)](https://github.com/user-attachments/assets/ad96720e-8dee-4e81-adbd-ac370bd0c646)

### Merge sessions

1. Click `Merge sessions`
2. Choose the default registration parameter set (`Indoor` or `Outdoor`) and click `OK`
3. Align point clouds using automatic or manual alignment
    - Automatic alignment : `Run global registration` and check if the point clouds are roughly aligned
    - Manual alignment : Roughly align the source points (green) with the target points (red) by manipulating the Gizmo UI.
4. Click `Run fine registration` to perform ICP matching
5. Click `Create factor` to merge sessions

[<img src="https://github.com/user-attachments/assets/3fbba90b-0586-499e-b952-d17094bac2a7" width="350">](https://github.com/user-attachments/assets/3fbba90b-0586-499e-b952-d17094bac2a7)

### Fuse sessions (Global matching cost minimization)

1. Click `Find overlapping submaps` to create matching cost factors between overlapping submaps. This improves the global consistency between sessions.
2. Click `Optimize` several times to run optimization steps (Or click the check box to continuously run optimization)

| Merge result | Merge result (Colored by session ID) | Factor graph |
|---|---|---|
|[![Image](https://github.com/user-attachments/assets/cb932d32-05f0-4e88-b8d9-a026a01beb5b)](https://github.com/user-attachments/assets/cb932d32-05f0-4e88-b8d9-a026a01beb5b)|[![Image](https://github.com/user-attachments/assets/1851cabd-6700-488a-bd11-e53ae64ba1d7)](https://github.com/user-attachments/assets/1851cabd-6700-488a-bd11-e53ae64ba1d7)|[![Image](https://github.com/user-attachments/assets/d932e144-60ac-499c-bead-be6015457149)](https://github.com/user-attachments/assets/d932e144-60ac-499c-bead-be6015457149)|

