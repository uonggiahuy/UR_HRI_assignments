# M4.3 evidence index

Overall acceptance is **FAIL / physical arrival tolerance NOT VERIFIED**. Final
acceptance requires one A-to-B action, replacing the original three-goal target.

- `a_to_b.json`, `a_to_b.log`, `navigation_a_to_b.log`: one real final-session
  NavigateToPose SUCCEEDED, actual motion, seven time-aligned path/grid snapshots,
  stopped odometry and safety observations. Raw PASS reflects the recorder's
  original Nav2 execution checks, not complete physical acceptance.
- `acceptance.json`: physical residual 0.311614 m, reference allowance 0.15 m;
  final physical arrival inside configured 0.25 m tolerance is unverified.
- `*_a_to_b.json`, `parameters_a_to_b.yaml`, `*_a_to_b.yaml`: final-session
  localization/stability gates, native configuration and readiness. Premature
  initialization/observation failures remain in `*_premature.*`.
- `*_baseline.*`, `*_motion_candidate.*`, `*_sensor_candidate.*`: native baseline,
  rejected motion-noise candidate and laser-only candidate. Experiment-completed
  PASS is distinct from sustained convergence PASS. Earlier observer `observe.py`
  uses latest TF for scan endpoints; those moving geometry values are superseded.
- `rotation_aligned*`, `rotation_short_tf.*`, `stability_short_tf.*`: exact-scan-time
  measurement trials, including a stopped asynchronous-delivery failure. The
  shipped observer uses a bounded recent scan whose exact-time TF is available.
- `*_verified.*`, `refresh_current_measurement.txt`: first shipped-config restart,
  physical stationary failure, convergence through controlled motion and one real
  no-motion measurement after stopping. Early nested assessments used 0.12 m;
  `final_reference_assessments.json` recomputes all windows with final 0.15 m.
- `registration.json`, `mesh_registration.json`, `registration_candidate.json`,
  `geometry_provenance.json`, `visible_shelves.npz`, geometry research scripts:
  independently derived simulator/map geometry, unsuccessful candidate and scoped
  held-out validation. `mesh_registration.json` intentionally remains an unreviewed
  fit result; reviewed `registration.json` adds independent scans and uncertainty.
- `first_goal_cancelled.*`, `second_goal_cancelled.*`, `three_goals_final.*`,
  `navigation_verified.log`: earlier same-target trials canceled by recorder
  defects; the old filenames do not establish three-goal execution or acceptance.
  Native TF errors after the earlier AMCL timeout are preserved. Final A-to-B
  launch owners remained active throughout the action.
- `selection_isolation.*`: read-only planner/TF isolation supporting bounded
  candidate selection; no NavigateToPose was sent by that experiment.
- `build.txt`, `colcon_tests.txt`, `colcon_results.txt`: final targeted build and
  **195 tests, 0 errors, 0 failures, 0 skipped**. `baseline_tests.txt` records the
  original 168-test baseline. Targeted earlier runs are historical.
- `map_hashes.txt`, `final_empty_graph.txt`, runtime/native graph logs: original
  map preservation and final cleanup. Empty graph is a zero-byte truthful artifact.
  Teardown errors remain in base logs; no claim of error-free Gazebo shutdown.

`summarize.py` produces covariance/particle/navigation scientific plots and the
consolidated assessment from raw measurements. Run it in the sourced ROS Humble,
assignment and TF2 0.25.24 environment. Geometry research scripts use this
container's installed SDF/Fuel files and `/tmp/m4_3`; copy the retained working
inputs there for replay. They read the map and never overwrite it.

The reference is bound to the unchanged user's map and scoped to map
x∈[-4,4], y∈[-4,2]. It does not certify the whole warehouse. Images are standard
plots of real data; no fabricated simulator screenshot or generated video is
included. Copied text logs have trailing whitespace normalized, with errors
preserved. Full desktop capture remains private and was excluded from Git.
