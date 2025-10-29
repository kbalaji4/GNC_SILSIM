// EKF Interactive Plotter JavaScript
class EKFPlotter {
    constructor() {
        this.data = null;
        this.filteredData = null;
        this.fsmStates = new Set();
        this.currentPlot = null;
        
        this.init();
    }

    async init() {
        try {
            await this.loadData();
            this.populateFSMStates();
            this.setupEventListeners();
            this.updatePlot();
        } catch (error) {
            this.showError('Failed to load data: ' + error.message);
        }
    }

    async loadData() {
        try {
            const response = await fetch('../output/results.csv');
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const csvText = await response.text();
            this.data = this.parseCSV(csvText);
            
            if (this.data.length === 0) {
                throw new Error('No data found in CSV file');
            }
            
            console.log(`Loaded ${this.data.length} data points`);
        } catch (error) {
            console.error('Error loading data:', error);
            throw error;
        }
    }

    parseCSV(csvText) {
        try {
            const lines = csvText.trim().split('\n');
            if (lines.length < 2) {
                throw new Error('CSV file must have at least a header and one data row');
            }
            
            const headers = lines[0].split(',').map(h => h.trim());
            
            const data = [];
            const maxRows = 50000; // Limit to prevent memory issues
            const endRow = Math.min(lines.length, maxRows + 1);
            
            for (let i = 1; i < endRow; i++) {
                const values = lines[i].split(',');
                if (values.length !== headers.length) continue;
                
                const row = {};
                headers.forEach((header, index) => {
                    let value = values[index].trim();
                    
                    // Convert numeric values
                    if (header !== 'fsm' && value !== 'nan' && value !== '') {
                        const numValue = parseFloat(value);
                        row[header] = isNaN(numValue) ? value : numValue;
                    } else {
                        row[header] = value;
                    }
                });
                
                data.push(row);
            }
            
            if (lines.length > maxRows + 1) {
                console.warn(`CSV has ${lines.length - 1} rows, limiting to first ${maxRows} rows for performance`);
            }
            
            console.log(`Parsed ${data.length} rows from CSV`);
            return data;
        } catch (error) {
            console.error('Error parsing CSV:', error);
            throw error;
        }
    }

    populateFSMStates() {
        this.fsmStates.clear();
        this.data.forEach(row => {
            if (row.fsm && row.fsm !== 'nan') {
                this.fsmStates.add(row.fsm);
            }
        });
        
        console.log('FSM States found:', Array.from(this.fsmStates));
    }

    setupEventListeners() {
        // Auto-update plot when any filter changes
        document.getElementById('dataType').addEventListener('change', () => this.updatePlot());
        document.getElementById('fsmFrom').addEventListener('change', () => this.updatePlot());
        document.getElementById('fsmTo').addEventListener('change', () => this.updatePlot());
        document.getElementById('rawData').addEventListener('change', () => this.updatePlot());
    }

    filterData() {
        const dataType = document.getElementById('dataType').value;
        const fsmFrom = document.getElementById('fsmFrom').value;
        const fsmTo = document.getElementById('fsmTo').value;
        
        // Get FSM state order for range filtering
        const fsmOrder = [
            'STATE_IDLE', 'STATE_FIRST_BOOST', 'STATE_BURNOUT', 'STATE_COAST', 
            'STATE_APOGEE', 'STATE_DROGUE_DEPLOY', 'STATE_DROGUE', 'STATE_MAIN_DEPLOY', 
            'STATE_MAIN', 'STATE_LANDED', 'STATE_SUSTAINER_IGNITION', 'STATE_SECOND_BOOST', 
            'STATE_FIRST_SEPARATION'
        ];
        
        const fromIndex = fsmOrder.indexOf(fsmFrom);
        const toIndex = fsmOrder.indexOf(fsmTo);
        
        this.filteredData = this.data.filter(row => {
            // Filter by FSM state range
            if (row.fsm && row.fsm !== 'nan') {
                const stateIndex = fsmOrder.indexOf(row.fsm);
                if (stateIndex < fromIndex || stateIndex > toIndex) {
                    return false;
                }
            }
            
            // Filter out NaN values for the selected data type
            if (isNaN(row[dataType]) || row[dataType] === 'nan') {
                return false;
            }
            
            return true;
        });
        
        console.log(`Filtered to ${this.filteredData.length} points (${fsmFrom} to ${fsmTo})`);
    }

    updatePlot() {
        this.filterData();
        
        const dataType = document.getElementById('dataType').value;
        const rawDataType = document.getElementById('rawData').value;
        
        // Get FSM state order for range filtering
        const fsmOrder = [
            'STATE_IDLE', 'STATE_FIRST_BOOST', 'STATE_BURNOUT', 'STATE_COAST', 
            'STATE_APOGEE', 'STATE_DROGUE_DEPLOY', 'STATE_DROGUE', 'STATE_MAIN_DEPLOY', 
            'STATE_MAIN', 'STATE_LANDED', 'STATE_SUSTAINER_IGNITION', 'STATE_SECOND_BOOST', 
            'STATE_FIRST_SEPARATION'
        ];
        
        const fsmFrom = document.getElementById('fsmFrom').value;
        const fsmTo = document.getElementById('fsmTo').value;
        const fromIndex = fsmOrder.indexOf(fsmFrom);
        const toIndex = fsmOrder.indexOf(fsmTo);
        
        if (this.filteredData.length === 0) {
            this.showError('No data points match the current filter criteria');
            return;
        }
        
        const traces = [];
        
        // Main EKF data trace
        const mainTrace = {
            x: this.filteredData.map(row => row.timestamp),
            y: this.filteredData.map(row => row[dataType]),
            type: 'scatter',
            mode: 'lines+markers',
            name: this.getDataTypeLabel(dataType),
            line: {
                color: '#667eea',
                width: 2
            },
            marker: {
                size: 3,
                color: '#667eea'
            }
        };
        traces.push(mainTrace);
        
        // Raw data trace (if selected) - filtered to same FSM range
        if (rawDataType !== 'none' && this.data[0][rawDataType] !== undefined) {
            const rawData = this.data.filter(row => {
                // Apply same FSM filtering as main data
                if (row.fsm && row.fsm !== 'nan') {
                    const stateIndex = fsmOrder.indexOf(row.fsm);
                    if (stateIndex < fromIndex || stateIndex > toIndex) {
                        return false;
                    }
                }
                
                // Filter out NaN values
                return !isNaN(row[rawDataType]) && row[rawDataType] !== 'nan';
            });
            
            if (rawData.length > 0) {
                const rawTrace = {
                    x: rawData.map(row => row.timestamp),
                    y: rawData.map(row => {
                        // Convert raw accelerometer data to m/s²
                        if (rawDataType.startsWith('raw_highg')) {
                            return row[rawDataType] * 9.81;
                        }
                        return row[rawDataType];
                    }),
                    type: 'scatter',
                    mode: 'lines',
                    name: this.getDataTypeLabel(rawDataType),
                    line: {
                        color: '#e74c3c',
                        width: 1,
                        dash: 'dash'
                    },
                    opacity: 0.7
                };
                traces.push(rawTrace);
            }
        }
        
        // FSM state change markers
        const fsmChanges = this.getFSMChanges();
        if (fsmChanges.length > 0) {
            const fsmTrace = {
                x: fsmChanges.map(change => change.timestamp),
                y: fsmChanges.map(change => change.y),
                type: 'scatter',
                mode: 'markers',
                name: 'FSM Changes',
                marker: {
                    size: 10,
                    color: '#f39c12',
                    symbol: 'diamond',
                    line: {
                        color: '#e67e22',
                        width: 2
                    }
                },
                text: fsmChanges.map(change => change.label),
                textposition: 'top center',
                textfont: {
                    size: 12,
                    color: '#2c3e50',
                    family: 'Arial, sans-serif'
                },
                hovertemplate: '<b>%{text}</b><br>' +
                              'Time: %{x:.2f}s<br>' +
                              '<extra></extra>'
            };
            traces.push(fsmTrace);
        }
        
        const layout = {
            title: {
                text: `${this.getDataTypeLabel(dataType)} vs Time`,
                font: { size: 20, color: '#2c3e50' }
            },
            xaxis: {
                title: 'Time (seconds)',
                gridcolor: '#e9ecef',
                gridwidth: 1
            },
            yaxis: {
                title: this.getYAxisLabel(dataType),
                gridcolor: '#e9ecef',
                gridwidth: 1
            },
            plot_bgcolor: 'white',
            paper_bgcolor: 'white',
            font: {
                family: 'Segoe UI, Tahoma, Geneva, Verdana, sans-serif',
                size: 12,
                color: '#2c3e50'
            },
            legend: {
                x: 0.02,
                y: 0.98,
                bgcolor: 'rgba(255,255,255,0.8)',
                bordercolor: '#e9ecef',
                borderwidth: 1
            },
            margin: { t: 60, r: 50, b: 60, l: 80 }
        };
        
        const config = {
            responsive: true,
            displayModeBar: true,
            modeBarButtonsToRemove: ['pan2d', 'lasso2d', 'select2d'],
            displaylogo: false
        };
        
        Plotly.newPlot('plot', traces, layout, config);
        this.currentPlot = { traces, layout, config };
        
        this.updateInfoPanel();
    }

    getFSMChanges() {
        const changes = [];
        let lastFSM = null;
        
        // Get FSM state order for range filtering
        const fsmOrder = [
            'STATE_IDLE', 'STATE_FIRST_BOOST', 'STATE_BURNOUT', 'STATE_COAST', 
            'STATE_APOGEE', 'STATE_DROGUE_DEPLOY', 'STATE_DROGUE', 'STATE_MAIN_DEPLOY', 
            'STATE_MAIN', 'STATE_LANDED', 'STATE_SUSTAINER_IGNITION', 'STATE_SECOND_BOOST', 
            'STATE_FIRST_SEPARATION'
        ];
        
        const fsmFrom = document.getElementById('fsmFrom').value;
        const fsmTo = document.getElementById('fsmTo').value;
        const fromIndex = fsmOrder.indexOf(fsmFrom);
        const toIndex = fsmOrder.indexOf(fsmTo);
        
        // Get the data range to calculate proper Y positioning
        const dataType = document.getElementById('dataType').value;
        const filteredData = this.data.filter(row => {
            if (row.fsm && row.fsm !== 'nan') {
                const stateIndex = fsmOrder.indexOf(row.fsm);
                if (stateIndex < fromIndex || stateIndex > toIndex) {
                    return false;
                }
            }
            return !isNaN(row[dataType]) && row[dataType] !== 'nan';
        });
        
        const values = filteredData.map(row => row[dataType]).filter(v => !isNaN(v));
        const maxValue = values.length > 0 ? Math.max(...values) : 1000;
        const minValue = values.length > 0 ? Math.min(...values) : 0;
        const range = maxValue - minValue;
        const yOffset = range * 0.1; // 10% above max value
        
        this.data.forEach((row, index) => {
            if (row.fsm && row.fsm !== 'nan' && row.fsm !== lastFSM) {
                // Only show FSM changes within the selected range
                const stateIndex = fsmOrder.indexOf(row.fsm);
                if (stateIndex >= fromIndex && stateIndex <= toIndex) {
                    changes.push({
                        timestamp: row.timestamp,
                        state: row.fsm,
                        label: this.getFSMStateLabel(row.fsm),
                        y: maxValue + yOffset
                    });
                }
                lastFSM = row.fsm;
            }
        });
        
        return changes;
    }

    getFSMStateLabel(fsmState) {
        const labels = {
            'STATE_IDLE': 'IDLE',
            'STATE_FIRST_BOOST': 'FIRST BOOST',
            'STATE_BURNOUT': 'BURNOUT',
            'STATE_COAST': 'COAST',
            'STATE_APOGEE': 'APOGEE',
            'STATE_DROGUE_DEPLOY': 'DROGUE DEPLOY',
            'STATE_DROGUE': 'DROGUE',
            'STATE_MAIN_DEPLOY': 'MAIN DEPLOY',
            'STATE_MAIN': 'MAIN',
            'STATE_LANDED': 'LANDED',
            'STATE_SUSTAINER_IGNITION': 'SUSTAINER IGNITION',
            'STATE_SECOND_BOOST': 'SECOND BOOST',
            'STATE_FIRST_SEPARATION': 'FIRST SEPARATION'
        };
        return labels[fsmState] || fsmState;
    }

    getDataTypeLabel(dataType) {
        const labels = {
            'pos_x': 'Position X',
            'pos_y': 'Position Y', 
            'pos_z': 'Position Z',
            'vel_x': 'Velocity X',
            'vel_y': 'Velocity Y',
            'vel_z': 'Velocity Z',
            'acc_x': 'Acceleration X',
            'acc_y': 'Acceleration Y',
            'acc_z': 'Acceleration Z',
            'altitude': 'Altitude',
            'raw_baro_alt': 'Raw Barometer Altitude',
            'raw_highg_ax': 'Raw HighG X',
            'raw_highg_ay': 'Raw HighG Y',
            'raw_highg_az': 'Raw HighG Z'
        };
        return labels[dataType] || dataType;
    }

    getYAxisLabel(dataType) {
        if (dataType.includes('pos') || dataType.includes('altitude')) {
            return 'Position (m)';
        } else if (dataType.includes('vel')) {
            return 'Velocity (m/s)';
        } else if (dataType.includes('acc') || dataType.includes('highg')) {
            return 'Acceleration (m/s²)';
        }
        return 'Value';
    }

    updateInfoPanel() {
        const dataType = document.getElementById('dataType').value;
        const fsmFrom = document.getElementById('fsmFrom').value;
        const fsmTo = document.getElementById('fsmTo').value;
        
        // Total data points
        document.getElementById('totalPoints').textContent = this.data.length.toLocaleString();
        
        // Filtered points
        document.getElementById('filteredPoints').textContent = this.filteredData.length.toLocaleString();
        
        // Flight duration
        const maxTime = Math.max(...this.data.map(row => row.timestamp));
        const minTime = Math.min(...this.data.map(row => row.timestamp));
        document.getElementById('flightDuration').textContent = `${(maxTime - minTime).toFixed(2)}s`;
        
        // Max/Min values
        const values = this.filteredData.map(row => row[dataType]).filter(v => !isNaN(v));
        if (values.length > 0) {
            document.getElementById('maxValue').textContent = Math.max(...values).toFixed(2);
            document.getElementById('minValue').textContent = Math.min(...values).toFixed(2);
        } else {
            document.getElementById('maxValue').textContent = 'N/A';
            document.getElementById('minValue').textContent = 'N/A';
        }
        
        // Current FSM range
        const currentFSM = fsmFrom === fsmTo ? fsmFrom : `${fsmFrom} to ${fsmTo}`;
        document.getElementById('currentFSM').textContent = currentFSM;
    }

    resetView() {
        if (this.currentPlot) {
            Plotly.relayout('plot', {
                'xaxis.autorange': true,
                'yaxis.autorange': true
            });
        }
    }

    showError(message) {
        const errorDiv = document.createElement('div');
        errorDiv.className = 'error';
        errorDiv.textContent = message;
        
        const plotContainer = document.getElementById('plot');
        plotContainer.innerHTML = '';
        plotContainer.appendChild(errorDiv);
    }
}

// Global functions for button clicks
function resetView() {
    if (window.plotter) {
        window.plotter.resetView();
    }
}

// Initialize the plotter when the page loads
document.addEventListener('DOMContentLoaded', function() {
    window.plotter = new EKFPlotter();
});
