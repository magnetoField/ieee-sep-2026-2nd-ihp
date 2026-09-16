// Auto-generated drone_dads_lvl12_s4 constants -- do not edit by hand.
// Emitted by train/train_sheila.py in tinytapeout-mnist-nn-asic as
// artifacts/headers/drone_dads_lvl12_s4.svh; the threshold was then moved to
// the validation-selected max-accuracy point (see below).
//
// drone_dads_lvl12, H=4 WL=1, nphase=2 shift=1 centre=8 nframe=16
// Trained on DADS with every clip re-levelled across a -12..0 dB span
// (extract_dads.py --relevel, pdm_gain 2.0); 12 seeds, seed 4 selected on
// validation AUC alone. Bit-exact chip model (eval_header.py --tag dads_lvl12
// --feat-off 8): validation AUC 99.15 %, test AUC 98.98 %.
// hidden template 276/320 non-zero (86%); zeros drop out of the adder tree.
localparam [639:0] WW_ROW      = 640'h15c5755c5714f57d5f57d4f53c7fd3f7fcff3fdf5f17c5f17c7f57d5f17d5f17d4f13c5f53f5f17cf0fffffccf7fdc771dc375dc775dd755d5755d544d5054d5357cdf17f5fd7f5fd7f5fc7fdff7f0f5;
localparam [23:0] WW_HBIAS    = 24'hfbdeb8;
localparam [7:0] WW_W2       = 8'hdf;
localparam [9:0] WW_THRESH_PK = 10'h005;
// Feature centre the template was trained at. The RTL subtracts FEAT_OFF from
// every band feature and asserts FEAT_OFF == WW_CENTRE at elaboration
// (A_CENTRE); a mismatch runs a different model from the one measured above.
localparam integer WW_CENTRE  = 8;
// hidden bias (decimal): [-8, -6, -3, -2]
// output weights: [-1, -1, 1, -1]   threshold: 5  (validation max-accuracy
// point, 96.1 % val accuracy; the trainer's 14 fired on 14 % of test drones)
