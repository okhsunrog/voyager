//! Voyager Interface Control Document - shared endpoint/topic definitions
//!
//! This crate defines the ergot endpoints and topics used for communication
//! between the Voyager firmware and host tools.

#![no_std]

use ergot::{endpoint, topic};

// Time service endpoints
endpoint!(SetTimeEndpoint, u64, (), "time/set");
endpoint!(GetTimeEndpoint, (), u64, "time/get");

// DFU endpoint
endpoint!(RebootToDfuEndpoint, (), (), "dfu/reboot");

// Topics for notifications
topic!(TimeUpdatedTopic, u64, "time/updated");
