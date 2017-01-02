#ifndef __UAPI_MEDIA_MSMB_GENERIC_BUF_MGR_H__
#define __UAPI_MEDIA_MSMB_GENERIC_BUF_MGR_H__

struct msm_buf_mngr_info {
	uint32_t session_id;
	uint32_t stream_id;
	uint32_t frame_id;
	struct timeval timestamp;
	uint32_t index;
	uint32_t reserved;
};

#define MSM_CAMERA_BUF_MNGR_IOCTL_ID_BASE 0
#define MSM_CAMERA_BUF_MNGR_IOCTL_ID_GET_BUF_BY_IDX 1

#define VIDIOC_MSM_BUF_MNGR_GET_BUF \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 33, struct msm_buf_mngr_info)

#define VIDIOC_MSM_BUF_MNGR_PUT_BUF \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 34, struct msm_buf_mngr_info)

#define VIDIOC_MSM_BUF_MNGR_BUF_DONE \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 35, struct msm_buf_mngr_info)

#define VIDIOC_MSM_BUF_MNGR_INIT \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 36, struct msm_buf_mngr_info)

#define VIDIOC_MSM_BUF_MNGR_DEINIT \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 37, struct msm_buf_mngr_info)

<<<<<<< HEAD
#ifdef CONFIG_COMPAT
struct msm_buf_mngr_info32_t {
	uint32_t session_id;
	uint32_t stream_id;
	uint32_t frame_id;
	struct compat_timeval timestamp;
	uint32_t index;
	uint32_t reserved;
};

#define VIDIOC_MSM_BUF_MNGR_GET_BUF32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 33, struct msm_buf_mngr_info32_t)

#define VIDIOC_MSM_BUF_MNGR_PUT_BUF32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 34, struct msm_buf_mngr_info32_t)

#define VIDIOC_MSM_BUF_MNGR_BUF_DONE32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 35, struct msm_buf_mngr_info32_t)
=======
#define VIDIOC_MSM_BUF_MNGR_IOCTL_CMD \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 40, \
	struct msm_camera_private_ioctl_arg)

>>>>>>> 0e91d2a... Nougat
#endif

