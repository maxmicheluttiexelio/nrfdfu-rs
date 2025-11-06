use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadioManifestEntry {
    pub bin_file: String,
    pub dat_file: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadioManifest {
    pub softdevice_bootloader: Option<RadioManifestEntry>,
    pub softdevice: Option<RadioManifestEntry>,
    pub bootloader: Option<RadioManifestEntry>,
    pub application: Option<RadioManifestEntry>,
}

impl RadioManifest {
    pub fn into_iter(self) -> RadioManifestIterator {
        RadioManifestIterator::new(self)
    }
}

pub struct RadioManifestIterator {
    radio_manifest: RadioManifest,
    index: usize,
}

impl RadioManifestIterator {
    pub fn new(radio_manifest: RadioManifest) -> Self {
        Self {
            radio_manifest,
            index: 0,
        }
    }
}

impl Iterator for RadioManifestIterator {
    type Item = Option<RadioManifestEntry>;

    fn next(&mut self) -> Option<Self::Item> {
        self.index += 1;
        match self.index {
            1 => Some(self.radio_manifest.softdevice_bootloader.clone()),
            2 => Some(self.radio_manifest.softdevice.clone()),
            3 => Some(self.radio_manifest.bootloader.clone()),
            4 => Some(self.radio_manifest.application.clone()),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadioManifestJSONObject {
    pub manifest: RadioManifest,
}
