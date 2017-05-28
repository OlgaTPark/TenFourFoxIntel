#ifdef 0
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */
#endif

/**
 * This singleton represents the whole 'New Tab Page' and takes care of
 * initializing all its components.
 */
let gPage = {
  /**
   * Initializes the page.
   */
  init: function Page_init() {
    // Add ourselves to the list of pages to receive notifications.
    gAllPages.register(this);

    // Listen for 'unload' to unregister this page.
    addEventListener("unload", this, false);

    // XXX bug 991111 - Not all click events are correctly triggered when
    // listening from xhtml nodes -- in particular middle clicks on sites, so
    // listen from the xul window and filter then delegate
    addEventListener("click", this, false);

    // Initialize sponsored panel
    this._sponsoredPanel = document.getElementById("sponsored-panel");
    let link = this._sponsoredPanel.querySelector(".text-link");
    link.addEventListener("click", () => this._sponsoredPanel.hidePopup());
    if (UpdateChannel.get().startsWith("release")) {
      document.getElementById("sponsored-panel-trial-descr").style.display = "none";
    }
    else {
      document.getElementById("sponsored-panel-release-descr").style.display = "none";
    }

    // Check if the new tab feature is enabled.
    let enabled = gAllPages.enabled;
    if (enabled)
      this._init();

    this._updateAttributes(enabled);
  },

  /**
   * True if the page is allowed to capture thumbnails using the background
   * thumbnail service.
   */
  get allowBackgroundCaptures() {
return false; // not yet supported
    // The preloader is bypassed altogether for private browsing windows, and
    // therefore allow-background-captures will not be set.  In that case, the
    // page is not preloaded and so it's visible, so allow background captures.
    return inPrivateBrowsingMode() ||
           document.documentElement.getAttribute("allow-background-captures") ==
           "true";
  },

  /**
   * Listens for notifications specific to this page.
   */
  observe: function Page_observe(aSubject, aTopic, aData) {
    if (aTopic == "nsPref:changed") {
      let enabled = gAllPages.enabled;
      this._updateAttributes(enabled);

      // Initialize the whole page if we haven't done that, yet.
      if (enabled) {
        this._init();
      } else {
        gUndoDialog.hide();
      }
    } else if (aTopic == "page-thumbnail:create" && gGrid.ready) {
      for (let site of gGrid.sites) {
        if (site && site.url === aData) {
          site.refreshThumbnail();
        }
      }
    }
  },

  /**
   * Updates the whole page and the grid when the storage has changed.
   * @param aOnlyIfHidden If true, the page is updated only if it's hidden in
   *                      the preloader.
   */
  update: function Page_update(aOnlyIfHidden=false) {
    let skipUpdate = aOnlyIfHidden && this.allowBackgroundCaptures;
    // The grid might not be ready yet as we initialize it asynchronously.
    if (gGrid.ready && !skipUpdate) {
      gGrid.refresh();
    }
  },

  /**
   * Shows sponsored panel
   */
  showSponsoredPanel: function Page_showSponsoredPanel(aTarget) {
    if (this._sponsoredPanel.state == "closed") {
      let self = this;
      this._sponsoredPanel.addEventListener("popuphidden", function onPopupHidden(aEvent) {
        self._sponsoredPanel.removeEventListener("popuphidden", onPopupHidden, false);
        aTarget.removeAttribute("panelShown");
      });
    }
    aTarget.setAttribute("panelShown", "true");
    this._sponsoredPanel.openPopup(aTarget);
  },

  /**
   * Internally initializes the page. This runs only when/if the feature
   * is/gets enabled.
   */
  _init: function Page_init() {
    if (this._initialized)
      return;

    this._initialized = true;

    gSearch.init();

    this._mutationObserver = new MutationObserver(() => {
      if (this.allowBackgroundCaptures) {
        Services.telemetry.getHistogramById("NEWTAB_PAGE_SHOWN").add(true);

        // Initialize type counting with the types we want to count
        let directoryCount = {};
        for (let type of DirectoryLinksProvider.linkTypes) {
          directoryCount[type] = 0;
        }

        for (let site of gGrid.sites) {
          if (site) {
            site.captureIfMissing();
            let {type} = site.link;
            if (type in directoryCount) {
              directoryCount[type]++;
            }
          }
        }

        // Record how many directory sites were shown, but place counts over the
        // default 9 in the same bucket
        for (let [type, count] of Iterator(directoryCount)) {
          let shownId = "NEWTAB_PAGE_DIRECTORY_" + type.toUpperCase() + "_SHOWN";
          let shownCount = Math.min(10, count);
          Services.telemetry.getHistogramById(shownId).add(shownCount);
        }

        // content.js isn't loaded for the page while it's in the preloader,
        // which is why this is necessary.
        gSearch.setUpInitialState();
      }
    });
    this._mutationObserver.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ["allow-background-captures"],
    });

    gLinks.populateCache(function () {
      // Initialize and render the grid.
      gGrid.init();

      // Initialize the drop target shim.
      gDropTargetShim.init();

#ifdef XP_MACOSX
      // Workaround to prevent a delay on MacOSX due to a slow drop animation.
      document.addEventListener("dragover", this, false);
      document.addEventListener("drop", this, false);
#endif
    }.bind(this));
  },

  /**
   * Updates the 'page-disabled' attributes of the respective DOM nodes.
   * @param aValue Whether the New Tab Page is enabled or not.
   */
  _updateAttributes: function Page_updateAttributes(aValue) {
    // Set the nodes' states.
    let nodeSelector = "#newtab-scrollbox, #newtab-toggle, #newtab-grid, #newtab-search-container";
    for (let node of document.querySelectorAll(nodeSelector)) {
      if (aValue)
        node.removeAttribute("page-disabled");
      else
        node.setAttribute("page-disabled", "true");
    }

    // Enables/disables the control and link elements.
    let inputSelector = ".newtab-control, .newtab-link";
    for (let input of document.querySelectorAll(inputSelector)) {
      if (aValue) 
        input.removeAttribute("tabindex");
      else
        input.setAttribute("tabindex", "-1");
    }

    // Update the toggle button's title.
    let toggle = document.getElementById("newtab-toggle");
    toggle.setAttribute("title", newTabString(aValue ? "hide" : "show"));
  },

  /**
   * Handles all page events.
   */
  handleEvent: function Page_handleEvent(aEvent) {
    switch (aEvent.type) {
      case "unload":
        if (this._mutationObserver)
          this._mutationObserver.disconnect();
        gAllPages.unregister(this);
        break;
      case "click":
        let {button, target} = aEvent;
        if (target.id == "newtab-toggle") {
          if (button == 0) {
            gAllPages.enabled = !gAllPages.enabled;
          }
          break;
        }

        // Go up ancestors until we find a Site or not
        while (target) {
          if (target.hasOwnProperty("_newtabSite")) {
            target._newtabSite.onClick(aEvent);
            break;
          }
          target = target.parentNode;
        }
        break;
      case "dragover":
        if (gDrag.isValid(aEvent) && gDrag.draggedSite)
          aEvent.preventDefault();
        break;
      case "drop":
        if (gDrag.isValid(aEvent) && gDrag.draggedSite) {
          aEvent.preventDefault();
          aEvent.stopPropagation();
        }
        break;
    }
  }
};
