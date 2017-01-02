/*
 * Copyright (c) 2013,2016 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "wil6210.h"
#include "trace.h"

<<<<<<< HEAD
int wil_err(struct wil6210_priv *wil, const char *fmt, ...)
=======
void __wil_err(struct wil6210_priv *wil, const char *fmt, ...)
>>>>>>> 0e91d2a... Nougat
{
	struct va_format vaf;
	va_list args;
	int ret;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
<<<<<<< HEAD
	ret = netdev_err(ndev, "%pV", &vaf);
=======
	netdev_err(wil_to_ndev(wil), "%pV", &vaf);
>>>>>>> 0e91d2a... Nougat
	trace_wil6210_log_err(&vaf);
	va_end(args);

	return ret;
}

<<<<<<< HEAD
int wil_info(struct wil6210_priv *wil, const char *fmt, ...)
=======
void __wil_info(struct wil6210_priv *wil, const char *fmt, ...)
>>>>>>> 0e91d2a... Nougat
{
	struct va_format vaf;
	va_list args;
	int ret;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
<<<<<<< HEAD
	ret = netdev_info(ndev, "%pV", &vaf);
=======
	netdev_info(wil_to_ndev(wil), "%pV", &vaf);
>>>>>>> 0e91d2a... Nougat
	trace_wil6210_log_info(&vaf);
	va_end(args);

	return ret;
}

void __wil_err_ratelimited(struct wil6210_priv *wil, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (!net_ratelimit())
		return;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	netdev_err(wil_to_ndev(wil), "%pV", &vaf);
	trace_wil6210_log_err(&vaf);
	va_end(args);
}

void wil_dbg_ratelimited(const struct wil6210_priv *wil, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (!net_ratelimit())
		return;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	netdev_dbg(wil_to_ndev(wil), "%pV", &vaf);
	trace_wil6210_log_dbg(&vaf);
	va_end(args);
}

int wil_dbg_trace(struct wil6210_priv *wil, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	trace_wil6210_log_dbg(&vaf);
	va_end(args);

	return 0;
}
