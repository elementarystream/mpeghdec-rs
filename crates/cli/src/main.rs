// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from demo/mpeghDecoder/main_mpeghDecoder.cpp.

//! Command line MPEG-H decoder (port of `demo/mpeghDecoder/main_mpeghDecoder.cpp`).
//!
//! Decodes the first MPEG-H track (`mhm1` or `mha1`) of an MP4 file, or a raw MHAS stream
//! (`.mhas`, with presentation timestamps from a `.pts.txt` side file if present), and writes the
//! rendered output as a 24 bit WAV file.

mod mhas;
mod mp4;
mod reader;
mod ui_manager_processor;
mod wav;

use mpeghdec::{Decoder, OutputInfo, Param, MAX_NUM_FRAME_SAMPLES};
use std::io::Write;
use std::process::ExitCode;

const MAX_RENDERED_CHANNELS: usize = 24;
const MAX_RENDERED_FRAME_SIZE: usize = MAX_NUM_FRAME_SAMPLES;

/// `WAV_BITS`
const WAV_BITS: u16 = 24;

const FDK_EXITCODE_OK: u8 = 0;
const FDK_EXITCODE_USAGE: u8 = 64;
const FDK_EXITCODE_SOFTWARE: u8 = 70;

/// `PARAMETER_ASSIGNMENT_TAB`
struct ParameterAssignment {
    param: Param,
    desc: &'static str,
    sw: &'static str,
}

static PARAM_LIST: [ParameterAssignment; 5] = [
    ParameterAssignment {
        param: Param::TargetReferenceLevel,
        desc: "DRC target loudness (reference level) in steps of -0.25 LU using values [40..127]",
        sw: "-rl",
    },
    ParameterAssignment {
        param: Param::EffectType,
        desc: "MPEG-D DRC effect type request, e.g.: 0 = None (default), 6 = General",
        sw: "-dse",
    },
    ParameterAssignment {
        param: Param::BoostFactor,
        desc: "DRC boost scale factor, where 0 is no boost and 127 is max boost",
        sw: "-db",
    },
    ParameterAssignment {
        param: Param::AttenuationFactor,
        desc: "DRC attenuation scale factor, where 0 is no compression and 127 is max compression",
        sw: "-dc",
    },
    ParameterAssignment {
        param: Param::AlbumMode,
        desc: "MPEG-D DRC: album mode, 0: disabled (default), 1: enabled",
        sw: "-dam",
    },
];

const DEFAULT_CICP_SETUP: i32 = 6;

/// `strtol(arg, NULL, 0)`: decimal, hexadecimal (`0x`) or octal (leading `0`) with optional sign;
/// parsing stops at the first invalid character.
fn strtol(s: &str) -> i32 {
    let s = s.trim_start();
    let (neg, s) = match s.as_bytes().first() {
        Some(b'-') => (true, &s[1..]),
        Some(b'+') => (false, &s[1..]),
        _ => (false, s),
    };
    let (radix, digits) = if s.starts_with("0x") || s.starts_with("0X") {
        (16, &s[2..])
    } else if s.starts_with('0') && s.len() > 1 {
        (8, &s[1..])
    } else {
        (10, s)
    };
    /* strtol() saturates to the range of the 64 bit long, the (INT) cast wraps */
    let mut v: i128 = 0;
    for c in digits.chars() {
        let Some(d) = c.to_digit(radix) else { break };
        v = (v * radix as i128 + d as i128).min(i128::from(i64::MAX) + 1);
    }
    let v = if neg { -v } else { v };
    v.clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64 as i32
}

/// The value following the switch `sw` (`IIS_ScanCmdl()`), if present.
fn arg_value<'a>(args: &'a [String], sw: &str) -> Option<&'a str> {
    let i = args.iter().position(|a| a == sw)?;
    args.get(i + 1).map(|s| s.as_str())
}

fn arg_int(args: &[String], sw: &str) -> Option<i32> {
    arg_value(args, sw).map(strtol)
}

/// One access unit of the input.
#[derive(Clone, Copy)]
struct Sample {
    /// Position of the payload in the input file.
    offset: u64,
    size: usize,
    pts: u64,
    duration: u64,
    is_sync: bool,
}

/// The MPEG-H track to decode. The payloads of the samples stay in the file ([`InputTrack::path`])
/// and are read on demand ([`InputTrack::reader`]).
struct InputTrack {
    /// The input file.
    path: String,
    samples: Vec<Sample>,
    timescale: u32,
    /// `mpegh3daConfig()` of an `mha1` track
    mha_config: Option<Vec<u8>>,
    max_sample_size: u32,
    sample_rate: u32,
}

impl InputTrack {
    /// A sample exists at `index` (an empty sample ends the track like in the demo).
    fn has_sample(&self, index: usize) -> bool {
        self.samples.get(index).is_some_and(|s| s.size > 0)
    }

    /// A reader of the sample payloads (every thread reading samples opens its own).
    fn reader(&self) -> Result<reader::SampleReader, String> {
        reader::SampleReader::open(&self.path).map_err(|e| format!("{}: {e}", self.path))
    }

    /// Index of the sync sample nearest to the sample `index` (mmtisobmff's
    /// `sampleByTimestamp()` in `nearestSyncSample` mode).
    ///
    /// For a nonexistent sample the result is the end of the track (an empty sample in
    /// mmtisobmff).
    fn nearest_sync_sample(&self, index: usize) -> usize {
        if index >= self.samples.len() {
            return self.samples.len();
        }
        /* the sample durations are accumulated from 0 (raw MHAS timestamps need not start at 0) */
        let seek_pts = self.samples[index].pts.saturating_sub(self.samples[0].pts);
        let seek_time = seek_pts as f64 / self.timescale as f64;
        let mut acc_duration: u64 = 0;
        let mut frame_index = 0usize;
        let mut sync_sample_index = 0usize;
        let mut sync_sample_index_n_minus_one = 0usize;
        let mut found = false;
        let mut user_seek_position_index = 0usize;
        for s in &self.samples {
            if s.is_sync {
                sync_sample_index_n_minus_one = sync_sample_index;
                sync_sample_index = frame_index;
                if found {
                    break;
                }
            }
            let current_time = acc_duration as f64 / self.timescale as f64;
            if current_time >= seek_time && !found {
                user_seek_position_index = frame_index;
                found = true;
            }
            acc_duration = acc_duration.saturating_add(s.duration);
            frame_index += 1;
        }
        if !found {
            user_seek_position_index = frame_index;
        }
        let d_sync = (sync_sample_index as i64 - user_seek_position_index as i64).abs();
        let d_prev = (user_seek_position_index as i64 - sync_sample_index_n_minus_one as i64).abs();
        if d_sync <= d_prev {
            sync_sample_index
        } else {
            sync_sample_index_n_minus_one
        }
    }
}

/// Opens the first MPEG-H track of an MP4 file (printing the track infos like the demo).
fn open_mp4(path: &str) -> Result<Option<InputTrack>, String> {
    let file = mp4::Mp4File::open(path).map_err(|e| e.to_string())?;
    println!("\nFound {} track(s) in input file.", file.tracks.len());
    let mut result = None;
    for track in &file.tracks {
        println!("########################################");
        println!("-TrackInfo: ");
        println!("-- ID       : {}", track.track_id);
        println!("-- Handler  : {}", String::from_utf8_lossy(&track.handler));
        println!("-- Codec    : {}", String::from_utf8_lossy(&track.coding_name));
        println!("-- Duration : {}", track.duration);
        println!("-- Timescale: {}", track.timescale);
        println!();

        if track.codec != mp4::Codec::MpeghMhm && track.codec != mp4::Codec::MpeghMha {
            println!("Skipping unsupported codec: {}", String::from_utf8_lossy(&track.coding_name));
            println!();
            continue;
        }
        if result.is_some() {
            println!("Skipping further mhm1 and mha1 tracks!");
            println!();
            continue;
        }
        if track.codec == mp4::Codec::MpeghMha {
            match &track.mha_config {
                None => {
                    println!("Warning: No MHA config available! Skipping track!");
                    continue;
                }
                Some(c) if c.is_empty() => {
                    println!("Warning: MHA config is empty! Skipping track!");
                    continue;
                }
                _ => {}
            }
        }

        let mut samples = Vec::with_capacity(track.samples.len());
        for s in &track.samples {
            if !file.sample_in_file(s) {
                return Err("sample data out of file bounds".to_string());
            }
            samples.push(Sample {
                offset: s.offset,
                size: s.size as usize,
                pts: s.pts(),
                duration: s.duration as u64,
                is_sync: s.is_sync,
            });
        }
        result = Some(InputTrack {
            path: path.to_string(),
            samples,
            timescale: track.timescale,
            mha_config: if track.codec == mp4::Codec::MpeghMha {
                track.mha_config.clone()
            } else {
                None
            },
            max_sample_size: track.max_sample_size(),
            sample_rate: track.sample_rate,
        });
    }
    Ok(result)
}

/// Opens a raw MHAS stream. The timestamps are taken from the `.pts.txt` side file (90 kHz) if
/// present, otherwise the access units are 1024 samples at 48 kHz apart. Access units carrying a
/// config packet are sync samples.
fn open_mhas(path: &str) -> Result<InputTrack, String> {
    let io = |e: std::io::Error| format!("{path}: {e}");
    let len = std::fs::metadata(path).map_err(io)?.len();
    let mut reader = reader::SampleReader::open(path).map_err(io)?;
    let base = path.strip_suffix(".mhas").unwrap_or(path);
    let pts_file = format!("{base}.pts.txt");
    let (aus, pts, timescale): (Vec<(u64, usize)>, Vec<u64>, u32) =
        if std::path::Path::new(&pts_file).exists() {
            let p = mhas::read_pts_file(&pts_file).map_err(|e| format!("{pts_file}: {e}"))?;
            (p.iter().map(|x| (x.0 as u64, x.1)).collect(), p.iter().map(|x| x.2).collect(), 90000)
        } else {
            let aus = mhas::split_access_units_reader(&mut reader, len).map_err(io)?;
            let pts = (0..aus.len() as u64).map(|i| i * 1024).collect();
            (aus, pts, 48000)
        };
    let mut samples = Vec::with_capacity(aus.len());
    for (i, &(offset, size)) in aus.iter().enumerate() {
        if offset.checked_add(size as u64).is_none_or(|e| e > len) {
            return Err(format!("{pts_file}: access unit {i} exceeds the stream"));
        }
        /* access units with a config packet are random access points */
        let is_sync = mhas::has_config_packet(reader.read(offset, size).map_err(io)?);
        let duration = match pts.get(i + 1) {
            Some(&n) => n.saturating_sub(pts[i]),
            None if i > 0 => pts[i].saturating_sub(pts[i - 1]),
            None => 1024,
        };
        samples.push(Sample { offset, size, pts: pts[i], duration, is_sync });
    }
    println!("\nFound 1 MHAS stream with {} access units in input file.", samples.len());
    let max_sample_size = samples.iter().map(|s| s.size as u32).max().unwrap_or(0);
    Ok(InputTrack {
        path: path.to_string(),
        samples,
        timescale,
        mha_config: None,
        max_sample_size,
        sample_rate: 48000,
    })
}

/// The number of output channels of the target layout `cicp_setup` (at least 1).
fn output_channels(cicp_setup: i32) -> usize {
    mpeghdec::num_output_channels(cicp_setup).max(1)
}

/// The default number of decoding threads (not in the demo): two threads overlap the decoding
/// and the rendering of consecutive frames, more gain little without chunked decoding.
const DEFAULT_THREADS: usize = 2;

/// `CProcessor`
struct Processor {
    wav_filename: String,
    wav_file: Option<wav::WavSink>,
    decoder: Decoder,
    cicp_setup: i32,
    /// The decoder parameters that were set (for the decoders of the chunked decoding).
    params: Vec<(Param, i32)>,
    /// State of the output loop (`process()`).
    out: OutputState,
}

/// The variables of the output loop of `process()`.
struct OutputState {
    /// Current samplerate
    sample_rate: u32,
    /// Current amount of output channels
    num_channels: i32,
    /// Audio output loudness
    output_loudness: i32,
    /// output frame counter
    frame_counter: u32,
}

/// The input track as the input of the chunked decoding: the samples up to the first empty one.
struct TrackSource<'a> {
    track: &'a InputTrack,
    len: usize,
}

/// Reads the samples of a [`TrackSource`].
struct TrackReader<'a>(&'a InputTrack, reader::SampleReader);

impl mpeghdec::chunked::AccessUnitReader for TrackReader<'_> {
    fn read(&mut self, index: usize) -> std::io::Result<&[u8]> {
        let s = &self.0.samples[index];
        self.1.read(s.offset, s.size)
    }
}

impl mpeghdec::chunked::AccessUnitSource for TrackSource<'_> {
    fn len(&self) -> usize {
        self.len
    }
    fn pts(&self, index: usize) -> u64 {
        self.track.samples[index].pts
    }
    fn size(&self, index: usize) -> usize {
        self.track.samples[index].size
    }
    fn timescale(&self) -> u32 {
        self.track.timescale
    }
    fn reader(&self) -> std::io::Result<Box<dyn mpeghdec::chunked::AccessUnitReader + Send + '_>> {
        Ok(Box::new(TrackReader(self.track, reader::SampleReader::open(&self.track.path)?)))
    }
}

impl Processor {
    fn new(output_filename: &str, cicp_setup: i32, num_threads: usize) -> Result<Self, String> {
        let decoder = Decoder::with_threads(cicp_setup, num_threads)
            .map_err(|_| "Error: Unable to create MPEG-H decoder".to_string())?;
        Ok(Processor {
            wav_filename: output_filename.to_string(),
            wav_file: None,
            decoder,
            cicp_setup,
            params: Vec::new(),
            out: OutputState {
                sample_rate: 0,
                num_channels: -1,
                output_loudness: -2,
                frame_counter: 0,
            },
        })
    }

    /// `configureDecoder()`: update decoder instance with all user parameters.
    fn configure_decoder(&mut self, args: &[String]) {
        for p in &PARAM_LIST {
            let Some(value) = arg_int(args, p.sw) else { continue };
            if self.decoder.set_param(p.param, value).is_err() {
                println!("Warning: Failed to set decoder parameter '{}' to {}", p.desc, value);
            } else {
                self.params.push((p.param, value));
            }
        }
        // fast mode (not in the reference demo): output not bit-exact
        if args.iter().skip(1).any(|a| a == "-fast") {
            self.decoder.set_fast_mode(true);
        }
    }

    /// The body of the output loop of `process()`: checks the output format and writes a frame
    /// (`samples`, interleaved) to the WAV file. `threaded`: write the file on a separate thread.
    fn output_frame(
        &mut self,
        out_info: &OutputInfo,
        samples: &[i32],
        threaded: bool,
    ) -> Result<(), String> {
        let st = &mut self.out;
        let frame_counter = st.frame_counter;
        if out_info.sample_rate != 48000 && out_info.sample_rate != 44100 {
            return Err("Error: Unsupported sampling rate".to_string());
        }
        if out_info.channels == 0 {
            return Err("Error: Unsupported number of channels".to_string());
        }
        if st.sample_rate != 0 && st.sample_rate != out_info.sample_rate {
            return Err("Error: Unsupported change of sampling rate".to_string());
        }
        let channels = out_info.channels as i32;
        if st.num_channels != -1 && st.num_channels != channels {
            return Err("Error: Unsupported change of number of channels".to_string());
        }
        let frame_size = out_info.samples_per_channel;
        st.sample_rate = out_info.sample_rate;
        st.num_channels = channels;

        let loudness = out_info.loudness.unwrap_or(-1);
        if st.output_loudness != loudness {
            st.output_loudness = loudness;
            let output_loudness = st.output_loudness;
            if output_loudness >= 0 {
                println!(
                    "[{frame_counter}] Info: audio output loudness {output_loudness} ({:.2} dBFS)",
                    -output_loudness as f32 * 0.25f32
                );
            } else {
                println!(
                    "[{frame_counter}] Info: audio output loudness {output_loudness}(no loudness metadata present)"
                );
            }
        }

        let (sample_rate, num_channels) = (st.sample_rate, st.num_channels);
        if self.wav_file.is_none() && !self.wav_filename.is_empty() && sample_rate != 0 {
            let w = wav::WavWriter::create(
                &self.wav_filename,
                sample_rate,
                num_channels as u16,
                WAV_BITS,
            )
            .map_err(|_| {
                format!(
                    "[{frame_counter}] Error: Unable to create output file {}",
                    self.wav_filename
                )
            })?;
            self.wav_file = Some(wav::WavSink::new(w, threaded));
        }

        if let Some(w) = self.wav_file.as_mut() {
            w.write_i32(&samples[..num_channels as usize * frame_size]).map_err(|_| {
                format!(
                    "[{frame_counter}] Error: Unable to write to output file {}",
                    self.wav_filename
                )
            })?;
        }
        self.out.frame_counter += 1;
        Ok(())
    }

    /// Chunked parallel decoding of the whole track (not in the demo, see
    /// [`mpeghdec::chunked`]): the same output as [`Processor::process`] without start, stop,
    /// seek and UI manager processing.
    fn process_chunked(
        &mut self,
        track: &InputTrack,
        opts: &mpeghdec::chunked::ChunkedOptions,
    ) -> Result<(), String> {
        use mpeghdec::chunked::{decode_chunked, ChunkedError, DecoderSetup};
        println!();
        println!("########################################");
        println!("ISOBMFF/MP4 Samples Info:");
        println!("Max ISOBMFF/MP4 Sample Size         : {} Bytes", track.max_sample_size);
        println!("Total number of ISOBMFF/MP4 Samples : {}", track.samples.len());
        println!();
        println!("########################################");
        let len = track.samples.iter().take_while(|s| s.size > 0).count();
        let src = TrackSource { track, len };
        let setup = DecoderSetup {
            cicp_setup: self.cicp_setup,
            params: self.params.clone(),
            mha_config: track.mha_config.clone(),
            fast: self.decoder.fast_options(),
        };
        println!(
            "Decoding all ISOBMFF/MP4 Samples of this track in {} chunks",
            opts.num_chunks(len, output_channels(self.cicp_setup))
        );
        let mut sink = |info: &OutputInfo, samples: &[i32]| self.output_frame(info, samples, true);
        let r = decode_chunked(&src, &setup, opts, &mut sink);
        println!();
        println!("Decoded MPEG-H audio frames: {}", self.out.frame_counter);
        match r {
            Ok(stats) => {
                if !setup.fast.is_exact() {
                    let n = stats.fast_bypass_frames;
                    println!("Fast mode: format converter bypassed in {n} frames");
                }
                Ok(())
            }
            Err(ChunkedError::Setup(e)) => Err(format!("Error: {e}")),
            Err(ChunkedError::Input { index, error }) => {
                Err(format!("[{index}] Error: Unable to read ISOBMFF/MP4 sample: {error}"))
            }
            Err(ChunkedError::Process { index, .. }) => {
                Err(format!("[{index}] Error: Unable to process data"))
            }
            Err(ChunkedError::Flush(_)) => Err("Error: Unable to flush data".to_string()),
            Err(ChunkedError::Output { frames, .. }) => {
                Err(format!("[{frames}] Error: Unable to obtain output"))
            }
            Err(ChunkedError::Sink(e)) => Err(e),
        }
    }

    /// `process()`. `ui` is the UI manager processing (`processUiManager`) applied to the access
    /// units before decoding.
    fn process(
        &mut self,
        track: &InputTrack,
        start_sample: i32,
        stop_sample: i32,
        seek_from_sample: i32,
        seek_to_sample: i32,
        mut ui: Option<&mut UiProcessing>,
    ) -> Result<(), String> {
        let mut out_data = vec![0i32; MAX_RENDERED_CHANNELS * MAX_RENDERED_FRAME_SIZE];

        if let Some(config) = &track.mha_config {
            if self.decoder.set_mha_config(config).is_err() {
                return Err("Error: Unable to set mpeghDecoder MHA configuration".to_string());
            }
            if ui.is_some() {
                println!(
                    "Warning: UI manager is not supported for MHA, ignoring interactivity script!"
                );
                ui = None;
            }
        }

        println!();
        println!("########################################");
        println!("ISOBMFF/MP4 Samples Info:");
        println!("Max ISOBMFF/MP4 Sample Size         : {} Bytes", track.max_sample_size);
        println!("Total number of ISOBMFF/MP4 Samples : {}", track.samples.len());
        println!();
        println!("########################################");
        println!("Reading all ISOBMFF/MP4 Samples of this track");

        let sample_count = track.samples.len() as u32;
        // check if enough samples are available to start at requested sample
        if start_sample >= 0 && start_sample as u32 >= sample_count {
            return Err(format!(
                "[0] Error: Too few ISOBMFF/MP4 Samples ({sample_count}) in track for starting at ISOBMFF/MP4 sample {start_sample}"
            ));
        }
        // check if enough samples are available to seek to requested sample
        if seek_to_sample >= 0 && seek_to_sample as u32 >= sample_count {
            return Err(format!(
                "[0] Error: Too few ISOBMFF/MP4 Samples ({sample_count}) in track for seeking to ISOBMFF/MP4 sample {seek_to_sample}"
            ));
        }

        // Get samples starting with the provided start sample
        let mut sample_counter: u32 = start_sample as u32;
        let mut index = track.nearest_sync_sample(sample_counter as usize);

        let mut seek_performed = false;
        let mut reader = track.reader()?;
        let mut ui_data = Vec::new();
        let mut stdout = std::io::stdout();
        let stdout_is_terminal = std::io::IsTerminal::is_terminal(&stdout);
        while track.has_sample(index) && sample_counter <= stop_sample as u32 {
            let s = track.samples[index];

            let mut data = reader.read(s.offset, s.size).map_err(|e| {
                format!("[{sample_counter}] Error: Unable to read ISOBMFF/MP4 sample: {e}")
            })?;
            if let Some(u) = ui.as_deref_mut() {
                ui_data.clear();
                ui_data.extend_from_slice(data);
                u.process(&mut ui_data, sample_counter as u64)?;
                data = &ui_data;
            }

            // Feed the sample data to the decoder.
            if self.decoder.process_timescale(data, s.pts, track.timescale).is_err() {
                return Err(format!("[{sample_counter}] Error: Unable to process data"));
            }

            sample_counter += 1;
            print!("ISOBMFF/MP4 Samples processed: {sample_counter}\r");
            if stdout_is_terminal {
                // Only a terminal needs the progress line immediately (saves a system call per
                // access unit when the output is redirected; the printed text is the same).
                let _ = stdout.flush();
            }

            if !seek_performed && sample_counter as i32 == seek_from_sample {
                println!(
                    "Performing seek from ISOBMFF/MP4 Sample {seek_from_sample} to ISOBMFF/MP4 Sample {seek_to_sample}"
                );
                if self.decoder.flush().is_err() {
                    return Err(format!("[{sample_counter}] Error: Unable to flush decoder"));
                }
                seek_performed = true;
                sample_counter = seek_to_sample as u32;
                index = track.nearest_sync_sample(sample_counter as usize);
            } else {
                // Get the next sample.
                index += 1;
            }

            // Check if EOF or the provided stop sample is reached.
            if !(track.has_sample(index) && sample_counter <= stop_sample as u32) {
                // Stop sample is reached. -> Flush the remaining output frames from the decoder.
                println!();
                println!("Flushing the decoder!");
                if self.decoder.flush_and_get().is_err() {
                    return Err("Error: Unable to flush data".to_string());
                }
            }

            // Obtain decoded audio frames.
            loop {
                let out_info = match self.decoder.get_samples(&mut out_data) {
                    Ok(Some(info)) => info,
                    Ok(None) => break,
                    Err(_) => {
                        return Err(format!(
                            "[{}] Error: Unable to obtain output",
                            self.out.frame_counter
                        ))
                    }
                };
                /* with several decoding threads the file is written by a separate thread */
                let threaded = self.decoder.threads() > 1;
                self.output_frame(&out_info, &out_data, threaded)?;
            }
        }

        println!();
        println!("Decoded MPEG-H audio frames: {}", self.out.frame_counter);
        if !self.decoder.fast_options().is_exact() {
            let n = self.decoder.fast_bypass_frames();
            println!("Fast mode: format converter bypassed in {n} frames");
        }
        let _ = track.sample_rate;
        Ok(())
    }

    fn close(mut self) -> Result<(), String> {
        if let Some(w) = self.wav_file.take() {
            w.close().map_err(|e| format!("Error: {}: {e}", self.wav_filename))?;
        }
        Ok(())
    }
}

/// The `processUiManager` callback of the demo: UI manager processing and the optional output of
/// the processed bitstream (`-bsof`, raw MHAS).
struct UiProcessing {
    processor: ui_manager_processor::UiManagerProcessor,
    bitstream_out: Option<(String, std::io::BufWriter<std::fs::File>)>,
}

impl UiProcessing {
    fn process(&mut self, sample: &mut Vec<u8>, sample_counter: u64) -> Result<(), String> {
        self.processor.process_single_sample(sample, sample_counter);
        if let Some((name, w)) = self.bitstream_out.as_mut() {
            // Write the UI-manager processed sample to the output
            w.write_all(sample).map_err(|e| format!("Error: {name}: {e}"))?;
        }
        Ok(())
    }

    fn finish(self) -> Result<(), String> {
        self.processor.finish();
        if let Some((name, mut w)) = self.bitstream_out {
            w.flush().map_err(|e| format!("Error: {name}: {e}"))?;
        }
        Ok(())
    }
}

fn cmdl_help(progname: &str) {
    println!();
    println!("Usage: {progname} [options] -if infile [-of outfile] [-bsof outfile]");
    println!("       options are:");
    println!("       -tl\tCICP index of the desired target layout (default: 6)");
    for p in &PARAM_LIST {
        println!("       {}\t{}", p.sw, p.desc);
    }
    println!(
        "       -y \tStart decoding at the provided ISOBMFF/MP4 sample number\n\
         \x20         \t  NOTE: The decoding will start at the nearest ISOBMFF/MP4 sync sample!\n\
         \x20      -z \tStop decoding at the provided ISOBMFF/MP4 sample number\n\
         \x20      -sf\tSeek in the bitstream from the provided ISOBMFF/MP4 sample number to the \n\
         \x20         \t  ISOBMFF/MP4 sample number provided with '-st'\n\
         \x20         \t  NOTE: '-st' must be set!\n\
         \x20      -st\tSeek in the bitstream from the ISOBMFF/MP4 sample number provided with '-sf'\n\
         \x20         \t  to the provided ISOBMFF/MP4 sample number\n\
         \x20         \t  NOTE: The decoding will resume at the nearest ISOBMFF/MP4 sync sample!\n\
         \x20         \t  NOTE: '-sf' must be set!\n\
         \n\
         \x20        \tSeeking example:\n\
         \x20        \t  '{progname} -if in.mp4 -of out.wav -sf 50 -st 100'\n\
         \x20        \t  will start decoding the input file from its first ISOBMFF/MP4 sample until\n\
         \x20        \t  ISOBMFF/MP4 sample 50 is reached.\n\
         \x20        \t  Afterwards it will seek to the nearest ISOBMFF/MP4 sync sample around\n\
         \x20        \t  ISOBMFF/MP4 sample 100 and resume decoding until the end of the input file\n\
         \x20        \t  is reached.\n\
         \x20      -script XML user interactivity script input file\n\
         \x20        \t  This file contains a list of XML ActionEvent tags with support for an additional\n\
         \x20        \t  '<Sleep Frame=\"<frame count>\" />' tag allowing to delay the successive ActionEvents\n\
         \x20        \t  by the given <frame count> of frames.\n\
         \x20      -xmlSceneState UI scene state output file.\n\
         \x20        \t  This file will contain a list of entries for every audio scene change, in the format\n\
         \x20        \t  '[<frame number>] <Audio Scene XML>' where <frame number> is the frame number at\n\
         \x20        \t  which the <Audio Scene XML> is first applied.\n\
         \x20      -persistFile Binary file to read/write UI persistency data from/to.\n\
         \x20        \t  NOTE: The contents of this file influence the UI manager behavior.\n\
         \x20      -j \tNumber of decoding threads (default: 2, at most the number of CPU cores;\n\
         \x20         \t  1 decodes single-threaded). The output does not depend on it.\n\
         \x20      -fast\tFast mode: skip the STFT format converter when it is equivalent to a delay\n\
         \x20         \t  (target layout = channel layout of the stream). The output is NOT bit-exact\n\
         \x20         \t  with the reference decoder (deviation below 4 LSB of 16 bit PCM).\n\
         \x20      -chunks\tChunked parallel decoding of the whole file with the given number of chunks\n\
         \x20         \t  (default: automatic for files of more than 2048 samples with -j 4 or more;\n\
         \x20         \t  0 decodes sequentially).\n\
         \x20         \t  Not used with -y, -z, -sf, -st and the UI manager options. The output is\n\
         \x20         \t  identical to the sequential decoding.\n\
         \x20      -cw \tWarm-up of a chunk in ISOBMFF/MP4 samples (default: 216)\n\
         \x20      -cv \tVerification points of a chunk boundary, comma-separated sample offsets\n\
         \x20         \t  from the chunk start (default: 0,64,192)\n\
         \x20      -cmem\tLimit of the buffered output of the chunked decoding in MiB (default: 256)\n\
         \x20      -cra\tRandom access points of the chunks: 'config' (default: samples with a\n\
         \x20         \t  configuration packet) or 'indep' (also independent frames)\n\
         \x20      -bsof \tOutput file for encoded bitstream processed by the MPEG-H UI Manager.\n\
         \x20        \t  The output file is written as raw MHAS stream (.mhas).\n\
         \x20      Raw MHAS input (.mhas) is supported as well: the access units are the sample\n\
         \x20        \t  numbers, timestamps are read from a '.pts.txt' side file if present.\n\
         \x20      -h\tShow this help"
    );
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let progname = args.first().map(|s| s.as_str()).unwrap_or("mpeghdec");

    // Check if helpMode was set.
    if args.iter().skip(1).any(|a| a == "-h") {
        cmdl_help(progname);
        return ExitCode::from(FDK_EXITCODE_OK);
    }

    // Check if we got the mandatory input and output parameters.
    let input_filename = arg_value(&args, "-if");
    let output_filename = arg_value(&args, "-of").unwrap_or("");
    let bitstream_filename = arg_value(&args, "-bsof").unwrap_or("");
    let Some(input_filename) = input_filename else {
        cmdl_help(progname);
        return ExitCode::from(FDK_EXITCODE_USAGE);
    };
    if output_filename.is_empty() && bitstream_filename.is_empty() {
        cmdl_help(progname);
        return ExitCode::from(FDK_EXITCODE_USAGE);
    }
    let script_filename = arg_value(&args, "-script").unwrap_or("");
    let xml_scene_state_filename = arg_value(&args, "-xmlSceneState").unwrap_or("");
    let persist_filename = arg_value(&args, "-persistFile").unwrap_or("");

    // Parse optional command line parameters
    let cicp_setup = arg_int(&args, "-tl").unwrap_or(DEFAULT_CICP_SETUP);
    let start_sample = arg_int(&args, "-y").unwrap_or(0);
    let stop_sample = arg_int(&args, "-z").unwrap_or(i32::MAX);
    let seek_from_sample = arg_int(&args, "-sf").unwrap_or(-1);
    let seek_to_sample = arg_int(&args, "-st").unwrap_or(-1);
    let available = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1);
    let num_threads = match arg_value(&args, "-j") {
        None => DEFAULT_THREADS.min(available),
        Some(v) => match v.parse::<usize>() {
            Ok(0) => DEFAULT_THREADS.min(available),
            Ok(n) => n,
            Err(_) => {
                cmdl_help(progname);
                return ExitCode::from(FDK_EXITCODE_USAGE);
            }
        },
    };

    // Chunked parallel decoding (not in the demo)
    let usize_arg = |sw: &str| -> Result<Option<usize>, ()> {
        match arg_value(&args, sw) {
            None => Ok(None),
            Some(v) => v.parse::<usize>().map(Some).map_err(|_| ()),
        }
    };
    let (Ok(chunks), Ok(chunk_warmup), Ok(chunk_mem)) =
        (usize_arg("-chunks"), usize_arg("-cw"), usize_arg("-cmem"))
    else {
        cmdl_help(progname);
        return ExitCode::from(FDK_EXITCODE_USAGE);
    };
    let log_state_differences = std::env::var_os("MPEGHDEC_CHUNK_DEBUG").is_some();
    let mut chunk_opts = mpeghdec::chunked::ChunkedOptions {
        threads: num_threads,
        chunks: chunks.unwrap_or(0),
        log: std::env::var_os("MPEGHDEC_CHUNK_LOG").is_some() || log_state_differences,
        log_state_differences,
        ..Default::default()
    };
    if let Some(w) = chunk_warmup {
        chunk_opts.warmup = w;
    }
    if let Some(m) = chunk_mem {
        chunk_opts.max_buffered_bytes = m << 20;
    }
    match arg_value(&args, "-cra") {
        None | Some("config") => {}
        Some("indep") => chunk_opts.independent_starts = true,
        Some(_) => {
            cmdl_help(progname);
            return ExitCode::from(FDK_EXITCODE_USAGE);
        }
    }
    if let Some(v) = arg_value(&args, "-cv") {
        let Ok(offsets) = v.split(',').map(|x| x.trim().parse::<usize>()).collect() else {
            cmdl_help(progname);
            return ExitCode::from(FDK_EXITCODE_USAGE);
        };
        chunk_opts.verify_offsets = offsets;
    }

    // Check if from and to sample for seeking are defined
    if (seek_from_sample != -1 && seek_to_sample == -1)
        || (seek_from_sample == -1 && seek_to_sample != -1)
    {
        cmdl_help(progname);
        return ExitCode::from(FDK_EXITCODE_USAGE);
    }

    println!("Input file:  {input_filename}");
    println!("Output file: {output_filename}");

    let run = || -> Result<(), String> {
        let mut ui = None;
        if !script_filename.is_empty()
            || !xml_scene_state_filename.is_empty()
            || !persist_filename.is_empty()
            || !bitstream_filename.is_empty()
        {
            let processor = ui_manager_processor::UiManagerProcessor::new(
                script_filename,
                persist_filename,
                xml_scene_state_filename,
            )?;
            let bitstream_out = if bitstream_filename.is_empty() {
                None
            } else {
                if !bitstream_filename.ends_with(".mhas") {
                    return Err(
                        "Error: The processed bitstream (-bsof) can only be written as raw MHAS (.mhas)"
                            .to_string(),
                    );
                }
                let f = std::fs::File::create(bitstream_filename)
                    .map_err(|e| format!("Error: {bitstream_filename}: {e}"))?;
                Some((bitstream_filename.to_string(), std::io::BufWriter::new(f)))
            };
            ui = Some(UiProcessing { processor, bitstream_out });
        }
        // initialize
        let mut processor = Processor::new(output_filename, cicp_setup, num_threads)?;
        // configure decoder
        processor.configure_decoder(&args);
        // process
        let track = if input_filename.ends_with(".mhas") {
            Some(open_mhas(input_filename)?)
        } else {
            open_mp4(input_filename)?
        };
        if let Some(track) = track {
            /* chunked parallel decoding of the whole file, unless an option needs the sequential
            decoding (start, stop, seek, UI manager processing) */
            let sequential_only = ui.is_some()
                || start_sample != 0
                || stop_sample != i32::MAX
                || seek_from_sample != -1;
            let len = track.samples.iter().take_while(|s| s.size > 0).count();
            if sequential_only && chunks.is_some_and(|c| c > 0) {
                println!(
                    "Note: chunked decoding is not used with -y, -z, -sf, -st and the UI manager options"
                );
            }
            if chunks != Some(0)
                && !sequential_only
                && chunk_opts.num_chunks(len, output_channels(cicp_setup)) > 1
            {
                processor.process_chunked(&track, &chunk_opts)?;
            } else {
                processor.process(
                    &track,
                    start_sample,
                    stop_sample,
                    seek_from_sample,
                    seek_to_sample,
                    ui.as_mut(),
                )?;
            }
        }
        if let Some(u) = ui {
            u.finish()?;
        }
        processor.close()
    };

    match run() {
        Ok(()) => ExitCode::from(FDK_EXITCODE_OK),
        Err(e) => {
            println!();
            println!("Error: {e}");
            println!();
            ExitCode::from(FDK_EXITCODE_SOFTWARE)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn samples_dir() -> Option<PathBuf> {
        let dir =
            std::env::var_os("MPEGHDEC_TEST_STREAMS").map(PathBuf::from).unwrap_or_else(|| {
                std::env::var_os("MPEGHDEC_WORK_DIR")
                    .map(PathBuf::from)
                    .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../work"))
                    .join("streams")
            });
        dir.is_dir().then_some(dir)
    }

    #[test]
    fn strtol_like_c() {
        assert_eq!(strtol("12"), 12);
        assert_eq!(strtol("-1"), -1);
        assert_eq!(strtol("0x10"), 16);
        assert_eq!(strtol("010"), 8);
        assert_eq!(strtol("7abc"), 7);
        assert_eq!(strtol("3000000000"), -1294967296);
        assert_eq!(strtol("4294967296"), 0);
        assert_eq!(strtol("99999999999999999999"), -1);
    }

    /// The MP4 sample tables and the MHAS splitter agree with the `.pts.txt` side files.
    #[test]
    fn mp4_and_mhas_match_pts_files() {
        let Some(dir) = samples_dir() else { return };
        for name in ["s1_clip_a", "s3_dvs_a", "s4_dvs_b"] {
            let base = dir.join(name);
            let mhas_path = base.with_extension("mhas");
            if !mhas_path.exists() {
                continue;
            }
            let pts =
                mhas::read_pts_file(base.with_extension("pts.txt").to_str().unwrap()).unwrap();
            let raw = std::fs::read(&mhas_path).unwrap();
            let aus = mhas::split_access_units(&raw);
            assert_eq!(aus.len(), pts.len(), "{name}: AU count");
            for (a, p) in aus.iter().zip(&pts) {
                assert_eq!((a.0, a.1), (p.0, p.1), "{name}: AU boundaries");
            }
            let mp4 = mp4::Mp4File::open(base.with_extension("mp4").to_str().unwrap()).unwrap();
            let track = &mp4.tracks[0];
            assert_eq!(track.codec, mp4::Codec::MpeghMhm);
            assert_eq!(track.samples.len(), pts.len(), "{name}: sample count");
            for (i, (s, p)) in track.samples.iter().zip(&pts).enumerate() {
                assert_eq!(mp4.read_sample(s).unwrap(), &raw[p.0..p.0 + p.1], "{name}: sample {i}");
                assert_eq!(s.pts(), i as u64 * 1024);
            }
            assert!(track.samples.iter().any(|s| s.is_sync));
            assert_eq!(track.nearest_sync_sample(0, track.timescale), 0);
        }
    }

    /// The streaming MHAS splitter agrees with the in-memory one, also for truncated streams.
    #[test]
    fn mhas_reader_splitter_matches() {
        let Some(dir) = samples_dir() else { return };
        let path = dir.join("s3_dvs_a.mhas");
        if !path.exists() {
            return;
        }
        let raw = std::fs::read(&path).unwrap();
        let tmp = std::env::temp_dir().join(format!("mpeghdec-split-{}.mhas", std::process::id()));
        for cut in [raw.len(), raw.len() - 1, raw.len() / 2 + 3, 5000, 7, 1, 0] {
            std::fs::write(&tmp, &raw[..cut]).unwrap();
            let mut r = reader::SampleReader::open(tmp.to_str().unwrap()).unwrap();
            let got = mhas::split_access_units_reader(&mut r, cut as u64).unwrap();
            let want: Vec<(u64, usize)> = mhas::split_access_units(&raw[..cut])
                .into_iter()
                .map(|(o, n)| (o as u64, n))
                .collect();
            assert_eq!(got, want, "cut at {cut}");
        }
        std::fs::remove_file(&tmp).unwrap();
    }

    /// Decoding with the samples read on demand gives the same output as decoding from the
    /// whole file in memory.
    #[test]
    fn on_demand_input_decodes_identically() {
        let Some(dir) = samples_dir() else { return };
        for name in ["s3_dvs_a.mp4", "s3_dvs_a.mhas"] {
            let path = dir.join(name);
            if !path.exists() {
                continue;
            }
            let path = path.to_str().unwrap();
            let track = if name.ends_with(".mhas") {
                open_mhas(path).unwrap()
            } else {
                open_mp4(path).unwrap().unwrap()
            };
            let whole = std::fs::read(path).unwrap();
            let decode = |from_file: bool| {
                let mut dec = Decoder::with_threads(2, 1).unwrap();
                let mut reader = track.reader().unwrap();
                let mut out = vec![0i32; MAX_RENDERED_CHANNELS * MAX_RENDERED_FRAME_SIZE];
                let mut pcm = Vec::new();
                for s in track.samples.iter().take(400) {
                    let data = if from_file {
                        reader.read(s.offset, s.size).unwrap()
                    } else {
                        &whole[s.offset as usize..s.offset as usize + s.size]
                    };
                    dec.process_timescale(data, s.pts, track.timescale).unwrap();
                    while let Some(info) = dec.get_samples(&mut out).unwrap() {
                        let n = info.samples();
                        pcm.extend_from_slice(&out[..n]);
                    }
                }
                pcm
            };
            let a = decode(true);
            assert!(!a.is_empty());
            assert!(a == decode(false), "{name}");
        }
    }

    #[test]
    fn wav_sample_conversion() {
        let mut x = 0x9e37_79b9_7f4a_7c15u64;
        let samples: Vec<i32> = (0..1003)
            .map(|i| {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                [x as i32, i32::MIN, i32::MAX, -1, 0, 0x80, -0x81][i % 7 * (i % 3) % 7]
            })
            .collect();
        for bits in [16u16, 24, 32] {
            let path = std::env::temp_dir()
                .join(format!("mpeghdec-wav-conv-{}-{bits}.wav", std::process::id()));
            let p = path.to_str().unwrap();
            let mut w = wav::WavWriter::create(p, 48000, 1, bits).unwrap();
            w.write_i32(&samples[..5]).unwrap();
            w.write_i32(&samples[5..]).unwrap();
            w.close().unwrap();
            let d = std::fs::read(p).unwrap();
            std::fs::remove_file(p).unwrap();
            let bytes = bits as usize / 8;
            let expect: Vec<u8> = samples
                .iter()
                .flat_map(|&s| (s >> (32 - bits)).to_le_bytes()[..bytes].to_vec())
                .collect();
            assert_eq!(&d[80..], &expect[..], "{bits} bits");
        }
    }

    #[test]
    fn wav_header_layout() {
        let path =
            std::env::temp_dir().join(format!("mpeghdec-wav-test-{}.wav", std::process::id()));
        let p = path.to_str().unwrap();
        let mut w = wav::WavWriter::create(p, 48000, 2, 24).unwrap();
        w.write_i32(&[0x1234_5678, -0x1234_5678, 0x7fff_ffff, i32::MIN]).unwrap();
        w.close().unwrap();
        let d = std::fs::read(p).unwrap();
        std::fs::remove_file(p).unwrap();
        assert_eq!(d.len(), 80 + 12);
        assert_eq!(&d[0..4], b"RIFF");
        assert_eq!(u32::from_le_bytes(d[4..8].try_into().unwrap()), 12 + 80 - 8);
        assert_eq!(&d[12..16], b"JUNK");
        assert_eq!(&d[76..80], &12u32.to_le_bytes());
        assert_eq!(&d[80..83], &[0x56, 0x34, 0x12]);
    }
}
