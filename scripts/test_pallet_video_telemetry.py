"""The annotated recording must fail when its own ROS telemetry is unusable."""
import unittest
from unittest.mock import patch

from record_pallet_bin_transfer import (OVERLAY_FIELDS, OverlayTelemetryAudit,
                                      OverlayTelemetryHistory, finalized_recording_error)


class VideoTelemetryAudit(unittest.TestCase):
    def state(self):
        result = {}
        for field in OVERLAY_FIELDS:
            result.update({field: 1.0, field + '_received': 99.9, field + '_sim_stamp': 20.0})
        return result

    def test_valid_synchronized_frame(self):
        audit = OverlayTelemetryAudit()
        self.assertIsNone(audit.check(self.state(), 100.0, 20.1))
        summary = audit.summary(True)
        self.assertTrue(summary['overlay_telemetry_passed'])
        self.assertEqual(summary['overlay_telemetry_audit']['simulation_sync_checks'], 4)

    def test_missing_nonfinite_stale_and_skewed_each_sensor_rejected(self):
        for field in OVERLAY_FIELDS:
            for mode in ('missing', 'nan', 'infinity', 'stale', 'skew'):
                with self.subTest(field=field, mode=mode):
                    state = self.state()
                    if mode == 'missing': state.pop(field)
                    if mode == 'nan': state[field] = float('nan')
                    if mode == 'infinity': state[field] = float('inf')
                    if mode == 'stale': state[field + '_received'] = 97.0
                    if mode == 'skew': state[field + '_sim_stamp'] = 22.0
                    audit = OverlayTelemetryAudit()
                    audit.failure = audit.check(state, 100.0, 20.0)
                    self.assertIsNotNone(audit.failure)
                    self.assertFalse(audit.summary(True)['overlay_telemetry_passed'])

    def test_clean_video_and_empty_recording_cannot_claim_overlay(self):
        audit = OverlayTelemetryAudit()
        self.assertFalse(audit.summary(True)['overlay_telemetry_passed'])
        self.assertIsNone(audit.check(self.state(), 100.0, 20.0))
        self.assertFalse(audit.summary(False)['overlay_telemetry_passed'])

    def test_missing_zero_and_nonfinite_sim_stamps_rejected(self):
        for field in OVERLAY_FIELDS:
            for stamp in (None, 0.0, -1.0, float('nan'), float('inf')):
                with self.subTest(field=field, stamp=stamp):
                    state = self.state()
                    state[field + '_sim_stamp'] = stamp
                    self.assertIsNotNone(OverlayTelemetryAudit().check(state, 100.0, 20.0))

    def test_clock_sample_after_snapshot_avoids_new_callback_race(self):
        state = self.state()
        for field in OVERLAY_FIELDS: state[field + '_received'] = 100.001
        with patch('record_pallet_bin_transfer.time.monotonic', return_value=100.002):
            self.assertIsNone(OverlayTelemetryAudit().check(state, None, 20.0))

    def test_delayed_frame_uses_historical_values_and_fresh_stream_separately(self):
        history = OverlayTelemetryHistory()
        historical = self.state()
        for field in OVERLAY_FIELDS:
            historical[field + '_received'] = 95.0
            history.append(field, historical)
        latest = self.state()
        for field in OVERLAY_FIELDS:
            latest[field] = 7.0
            latest[field + '_sim_stamp'] = 23.0
            history.append(field, latest)
        selected = history.at_sim_time(latest, 20.1)
        self.assertEqual(selected['waist_pitch_deg'], 1.0)
        audit = OverlayTelemetryAudit()
        self.assertIsNone(audit.check(selected, 100.0, 20.1, latest_state=latest))
        # Historical selection must never conceal a stopped or corrupt live feed.
        latest['left_force_norm_n_received'] = 97.0
        self.assertIsNotNone(audit.check(selected, 100.0, 20.1, latest_state=latest))
        latest['left_force_norm_n_received'] = 99.9
        latest['left_force_norm_n_sim_stamp'] = float('nan')
        self.assertIsNotNone(audit.check(selected, 100.0, 20.1, latest_state=latest))

    def test_history_exhaustion_and_invalid_image_stamp_rejected(self):
        history = OverlayTelemetryHistory(max_samples=1)
        state = self.state()
        for field in OVERLAY_FIELDS:
            history.append(field, state)
            state[field + '_sim_stamp'] = 23.0
            history.append(field, state)
        selected = history.at_sim_time(state, 20.0)
        self.assertIsNotNone(OverlayTelemetryAudit().check(selected, 100.0, 20.0, latest_state=state))
        self.assertIsNotNone(OverlayTelemetryAudit().check(self.state(), 100.0, float('nan')))

    def test_startup_readiness_does_not_count_encoded_frame(self):
        audit = OverlayTelemetryAudit()
        self.assertIsNone(audit.check(self.state(), 100.0, 20.0, count_frame=False))
        self.assertFalse(audit.summary(True)['overlay_telemetry_passed'])

    def test_final_runner_gate_rejects_missing_overlay_clean_or_failed_recording(self):
        valid = dict(passed=True, recorded_completed_transfers=9, clean_video=False,
                     overlay_telemetry_passed=True)
        self.assertIsNone(finalized_recording_error(valid, 0))
        self.assertIsNotNone(finalized_recording_error(valid, 1))
        for field, value in (('passed', False), ('recorded_completed_transfers', 8),
                             ('clean_video', True), ('overlay_telemetry_passed', False)):
            self.assertIsNotNone(finalized_recording_error({**valid, field: value}, 0))
        valid.pop('overlay_telemetry_passed')
        self.assertIsNotNone(finalized_recording_error(valid, 0))


if __name__ == '__main__': unittest.main()
