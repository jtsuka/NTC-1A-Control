import sigrokdecode as srd

class Decoder(srd.Decoder):
    api_version = 3
    id = 'tc106_step4b'
    name = 'TC106 Step4B'
    longname = 'TC106 Step4B Direction-B telemetry'
    desc = 'Decodes provisional TC106/Nano -> Main/ESP 17-slot telemetry frames.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['tc106_step4b']
    tags = ['Embedded/industrial']

    channels = (
        {'id': 'data', 'name': 'DATA', 'desc': 'TC106/Nano Direction-B signal'},
    )

    options = (
        {'id': 'slot_us', 'desc': 'Nominal slot width (us)', 'default': 3300},
        {'id': 'tolerance_pct', 'desc': 'Timing tolerance (%)', 'default': 18},
        {'id': 'frame_bytes', 'desc': 'Telemetry frame bytes', 'default': 6},
        {'id': 'footer', 'desc': 'Expected footer byte (decimal)', 'default': 127},
        {'id': 'show_bits', 'desc': 'Show data-bit annotations',
         'default': 'yes', 'values': ('yes', 'no')},
        {'id': 'show_slots', 'desc': 'Show slot annotations',
         'default': 'no', 'values': ('yes', 'no')},
        {'id': 'polarity', 'desc': 'Observed Step4B polarity',
         'default': 'step4b', 'values': ('step4b', 'inverted')},
    )

    annotations = (
        ('slot', 'Slot'),
        ('bit', 'Data bit'),
        ('byte', 'Byte'),
        ('field', 'Telemetry field'),
        ('frame', 'Telemetry frame'),
        ('warning', 'Warning'),
        ('sync', 'Synchronization'),
    )

    annotation_rows = (
        ('slots', 'Slots', (0,)),
        ('bits', 'Bits', (1,)),
        ('bytes', 'Bytes', (2,)),
        ('fields', 'Fields', (3,)),
        ('frames', 'Frames', (4,)),
        ('warnings', 'Warnings', (5, 6)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.slot_samples = None
        self.out_ann = None
        self.last_level = 0
        self.capture_start = 0
        self.search_cursor = 0

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            self.slot_samples = max(
                1, int(round(value * float(self.options['slot_us']) / 1000000.0))
            )

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def _put(self, ss, es, ann, texts):
        self.put(int(ss), int(es), self.out_ann, [ann, texts])

    def _logical(self, raw):
        if self.options['polarity'] == 'step4b':
            return 1 if raw else 0
        return 0 if raw else 1

    def _sample_at(self, target):
        if target <= self.samplenum:
            return self.last_level
        pins = self.wait({'skip': int(target - self.samplenum)})
        self.last_level = pins[0]
        return self.last_level

    def _decode_candidate(self, frame_ss):
        slot = self.slot_samples
        frame_bytes = int(self.options['frame_bytes'])
        footer = int(self.options['footer']) & 0x7f
        vals = []
        byte_spans = []
        bit_spans = []
        slot_spans = []

        if self._logical(self._sample_at(frame_ss + slot // 2)) != 0:
            return None

        for bi in range(frame_bytes):
            base = frame_ss + bi * 17 * slot

            if self._logical(self._sample_at(base + slot // 2)) != 0:
                return None

            value = 0
            bits = []
            for bit in range(7):
                s = base + (1 + bit) * slot
                e = s + slot
                lev = self._logical(self._sample_at(s + slot // 2))
                if lev:
                    value |= (1 << bit)
                bits.append((bit, lev, s, e))

            for si in range(8, 17):
                s = base + si * slot
                lev = self._logical(self._sample_at(s + slot // 2))
                if lev != 1:
                    return None

            vals.append(value)
            bit_spans.append(bits)
            byte_spans.append((base, base + 17 * slot))

            if self.options['show_slots'] == 'yes':
                for si in range(17):
                    slot_spans.append(
                        (si, base + si * slot, base + (si + 1) * slot)
                    )

        if vals[-1] != footer:
            return None

        return (
            vals, byte_spans, bit_spans, slot_spans,
            frame_ss, frame_ss + frame_bytes * 17 * slot
        )

    def _emit(self, result):
        vals, byte_spans, bit_spans, slot_spans, ss, es = result

        if self.options['show_slots'] == 'yes':
            for si, s, e in slot_spans:
                self._put(s, e, 0, ['S%d' % si, str(si)])

        if self.options['show_bits'] == 'yes':
            for bits in bit_spans:
                for bit, lev, s, e in bits:
                    self._put(s, e, 1, ['b%d=%d' % (bit, lev), str(lev)])

        names = ('LEN0', 'LEN1', 'LEN2', 'TENS0', 'TENS1', 'END')
        for i, val in enumerate(vals):
            bss, bes = byte_spans[i]
            self._put(bss, bes, 2,
                      ['BYTE%d 0x%02X' % (i, val), '%02X' % val])
            label = names[i] if i < len(names) else 'B%d' % i
            if i == 5:
                self._put(bss, bes, 3, ['END 0x%02X' % val, 'END'])
            else:
                self._put(bss, bes, 3,
                          ['%s=0x%02X' % (label, val), label])

        summary = 'TELEMETRY %s END=0x%02X' % (
            ' '.join('%02X' % v for v in vals[:-1]), vals[-1]
        )
        self._put(ss, es, 4, [summary, 'TELEMETRY'])
        self._put(ss, ss + self.slot_samples, 6,
                  ['AUTO full-frame validation', 'SYNC'])

    def decode(self):
        if not self.samplerate or not self.slot_samples:
            raise srd.SamplerateError('Samplerate is required.')

        pins = self.wait({})
        self.capture_start = self.samplenum
        self.last_level = pins[0]
        self.search_cursor = self.capture_start

        while True:
            pins = self.wait({'data': 'e'})
            edge = self.samplenum
            self.last_level = pins[0]

            slot = self.slot_samples
            found = None

            # Step4B has no guaranteed edge at byte0/slot0.
            # Try candidate slot0 positions up to one byte before each edge,
            # and only accept candidates that validate the full 6-byte frame
            # and footer 0x7F.
            for k in range(18):
                cand = int(round(edge - k * slot))
                if cand < self.capture_start or cand < self.search_cursor:
                    continue
                if cand + slot // 2 <= self.samplenum:
                    continue
                result = self._decode_candidate(cand)
                if result:
                    found = result
                    break

            if found:
                self._emit(found)
                self.search_cursor = found[-1]
