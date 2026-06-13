import mighty_protocol as mp

class DecodedDispatcher:
    """
    Feed raw bytes; dispatches per type with decoded payloads.
    Handlers (all optional):
      on_jpg(timestamp_ns, channel, data, is_ref)
      on_raw(timestamp_ns, width, height, format, channel, data)
      on_stereo_raw(left_dict, right_dict)
      on_pose(pose_dict, is_unoptimized)
      on_constraints(segments_list)
      on_features(features_list)
      on_pointcloud(points, point_size)
      on_keyframe(keyframe_dict)
      on_viz(payload_bytes)
      on_imu(samples_list)
      on_status(text)
      on_vio_state(state_dict)
      on_reset()
      on_command(cmd_dict)
      on_command_response(cres_dict)
      on_config_request(cfgq_dict)
      on_config_response(cfgr_dict)
    """
    def __init__(self):
        self.on_jpg = None
        self.on_raw = None
        self.on_stereo_raw = None
        self.on_pose = None
        self.on_constraints = None
        self.on_features = None
        self.on_pointcloud = None
        self.on_keyframe = None
        self.on_viz = None
        self.on_imu = None
        self.on_status = None
        self.on_vio_state = None
        self.on_reset = None
        self.on_command = None
        self.on_command_response = None
        self.on_config_request = None
        self.on_config_response = None
        self._buffer = b""

    def feed(self, data: bytes):
        self._buffer += data
        frames, rest = mp.parse_frames(self._buffer)
        self._buffer = rest
        for f in frames:
            t = f["type"].strip()
            p = f["payload"]
            if t == "JPG":
                if self.on_jpg:
                    d = mp.decode_jpg_payload(p, False); self.on_jpg(d["timestamp_ns"], d["channel"], d["data"], False)
            elif t == "RJPG":
                if self.on_jpg:
                    d = mp.decode_jpg_payload(p, True); self.on_jpg(d["timestamp_ns"], d["channel"], d["data"], True)
            elif t == "RAW":
                if self.on_raw:
                    d = mp.decode_raw_payload(p)
                    self.on_raw(d["timestamp_ns"], d["width"], d["height"], d["format"], d["channel"], d["data"])
            elif t == "SRAW":
                if self.on_stereo_raw or self.on_raw:
                    d = mp.decode_stereo_raw_payload(p)
                    if self.on_stereo_raw:
                        self.on_stereo_raw(d["left"], d["right"])
                    elif self.on_raw:
                        left = d["left"]
                        right = d["right"]
                        self.on_raw(left["timestamp_ns"], left["width"], left["height"], left["format"], left["channel"], left["data"])
                        self.on_raw(right["timestamp_ns"], right["width"], right["height"], right["format"], right["channel"], right["data"])
            elif t == "POSE":
                if self.on_pose:
                    d = mp.decode_pose_payload(p); self.on_pose(d, False)
            elif t == "UPOS":
                if self.on_pose:
                    d = mp.decode_pose_payload(p); self.on_pose(d, True)
            elif t == "LCON":
                if self.on_constraints:
                    self.on_constraints(mp.decode_constraints_payload(p))
            elif t == "FEA3":
                if self.on_features:
                    self.on_features(mp.decode_fea3_payload(p))
            elif t == "PCLD":
                if self.on_pointcloud:
                    res = mp.decode_pcld_payload(p); self.on_pointcloud(res["points"], res["point_size"])
            elif t == "KEYF":
                if self.on_keyframe:
                    self.on_keyframe(mp.decode_keyframe_payload(p))
            elif t == "VIZ":
                if self.on_viz: self.on_viz(p)
            elif t == "IMU":
                if self.on_imu: self.on_imu(mp.decode_imu_payload(p))
            elif t == "STAT":
                if self.on_status: self.on_status(mp.decode_status_payload(p))
            elif t == "VSTA":
                if self.on_vio_state: self.on_vio_state(mp.decode_vio_state_payload(p))
            elif t == "RSET":
                if self.on_reset: self.on_reset()
            elif t == "CMD":
                if self.on_command: self.on_command(mp.decode_command_payload(p))
            elif t == "CRES":
                if self.on_command_response: self.on_command_response(mp.decode_command_response_payload(p))
            elif t == "CFGQ":
                if self.on_config_request: self.on_config_request(mp.decode_config_request_payload(p))
            elif t == "CFGR":
                if self.on_config_response: self.on_config_response(mp.decode_config_response_payload(p))
