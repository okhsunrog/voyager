//! Voyager Interface Control Document - shared endpoint/topic definitions
//!
//! This crate defines the ergot endpoints and topics used for communication
//! between the Voyager firmware and host tools.

#![no_std]

use ergot::{endpoint, topic};

// Time service endpoints (u32 timestamp valid until year 2106)
endpoint!(SetTimeEndpoint, u32, (), "time/set");
endpoint!(GetTimeEndpoint, (), u32, "time/get");

// DFU endpoint
endpoint!(RebootToDfuEndpoint, (), (), "dfu/reboot");

// Topics for notifications
topic!(TimeUpdatedTopic, u32, "time/updated");
